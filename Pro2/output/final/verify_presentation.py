"""Verify original-image reuse, embedded MP4s, chapter order and PDF rendering."""
from pathlib import Path
from zipfile import ZipFile
import argparse, hashlib, io, json, posixpath, re, subprocess, tempfile, unicodedata
from lxml import etree
from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
NS = {'a':'http://schemas.openxmlformats.org/drawingml/2006/main',
      'p':'http://schemas.openxmlformats.org/presentationml/2006/main',
      'r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships'}

def sha(data): return hashlib.sha256(data).hexdigest()
def norm(s): return re.sub(r'\s+', '', unicodedata.normalize('NFKC', s))
def texts(slide):
    result=[]
    for sh in slide.shapes:
        if sh.has_text_frame: result.append(sh.text)
        if sh.has_table:
            result.extend(c.text for row in sh.table.rows for c in row.cells)
    return result

def render(pdf, folder):
    folder.mkdir(exist_ok=True, parents=True)
    subprocess.run(['pdftoppm','-scale-to','1440','-png',str(pdf),str(folder/'slide')],check=True,timeout=120)
    files=sorted(folder.glob('slide-*.png'))
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',19)
    for start in range(0,len(files),8):
        sheet=Image.new('RGB',(1600,510),(226,232,235));draw=ImageDraw.Draw(sheet)
        for j,path in enumerate(files[start:start+8]):
            im=Image.open(path).convert('RGB');im.thumbnail((390,220))
            x=j%4*400;y=j//4*255
            sheet.paste(im,(x+5,y+29));draw.text((x+10,y+3),f'{start+j+1:02}',font=font,fill=(32,52,62))
        sheet.save(folder/f'contact-{start//8+1:02}.png')

def export_pdf(stems):
    # A PDF cannot play native PowerPoint media. Explicitly use each embedded
    # poster in a temporary export copy; leave the delivered PPTX untouched.
    with tempfile.TemporaryDirectory(prefix='pro2_pdf_') as temp:
        temp=Path(temp);files=[]
        for stem in stems:
            prs=Presentation(OUT/(stem+'.pptx'))
            for slide in prs.slides:
                for sh in list(slide.shapes):
                    if not sh._element.xpath('.//a:videoFile'):continue
                    media=sh._element;parent=media.getparent();idx=parent.index(media)
                    picture=slide.shapes.add_picture(io.BytesIO(sh.poster_frame.blob),sh.left,sh.top,sh.width,sh.height)
                    parent.remove(picture._element);parent.remove(media);parent.insert(idx,picture._element)
            target=temp/(stem+'.pptx');prs.save(target);files.append(str(target))
        subprocess.run(['libreoffice','-env:UserInstallation='+(temp/'profile').as_uri(),'--headless','--convert-to','pdf','--outdir',str(OUT),*files],check=True,timeout=120)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--render',action='store_true')
    parser.add_argument('--export-pdf',action='store_true');args=parser.parse_args()
    manifest=json.loads((OUT/'presentation_sources.json').read_text())
    if args.export_pdf:export_pdf(manifest['decks'])
    issues=[];decks=[];warnings=[]
    for stem,pages in manifest['decks'].items():
        file=OUT/(stem+'.pptx');prs=Presentation(file)
        if len(prs.slides)!=len(pages):issues.append(f'{stem}: page count')
        if len(prs.slides)!=15:issues.append(f'{stem}: required 15 pages')
        order=[int(r['stage'][1]) for r in pages if r['stage'].startswith('L')]
        if order!=sorted(order) or set(order)!=set(range(5)):issues.append(f'{stem}: stage order')
        images=0;videos=0;autoplay=0;tables=0;bounds=[];notes=0;inherited_bounds=[]
        baseline=None
        audit_paths=[OUT/'checks/source_link_update.json',OUT/'checks/terrain_analysis_update.json',OUT/'checks/cover_video_update.json',OUT/'checks/terrain_slide_update.json',OUT/'checks/page12_explanation_update.json',OUT/'checks/presentation_text_update.json',OUT/'checks/push_presentation_update.json']
        for audit_path in audit_paths:
            if not audit_path.is_file():continue
            audit=json.loads(audit_path.read_text())
            snapshot=Path(audit.get('snapshot_before',''))
            if snapshot.is_file() and sha(file.read_bytes())==audit.get('output_pptx_sha256') and sha(snapshot.read_bytes())==audit.get('input_pptx_sha256'):
                baseline=Presentation(snapshot)
                break
        with ZipFile(file) as z:
            bad=z.testzip()
            if bad:issues.append(f'{stem}: ZIP CRC {bad}')
            if any(n.startswith('ppt/charts/') for n in z.namelist()):issues.append(f'{stem}: redrawn native chart')
            for sl,record in zip(prs.slides,pages):
                num=record['page'];alltext='\n'.join(texts(sl))
                if re.search(r'L[0-4]\s*[-–]\s*[0-4]',alltext):issues.append(f'{stem} {num}: ambiguous level label')
                if any(x in alltext for x in ['\\begin{','\\[','\\(','\\frac']):issues.append(f'{stem} {num}: raw LaTeX')
                notes+=int('证据来源' in sl.notes_slide.notes_text_frame.text)
                for sh in sl.shapes:
                    tables+=int(sh.has_table)
                    if sh.left<0 or sh.top<0 or sh.left+sh.width>prs.slide_width+1000 or sh.top+sh.height>prs.slide_height+1000:
                        bounds.append([num,sh.name])
                        if baseline is not None:
                            previous=[item for item in baseline.slides[num-1].shapes if item.name==sh.name]
                            if len(previous)==1 and etree.tostring(previous[0]._element)==etree.tostring(sh._element):
                                inherited_bounds.append([num,sh.name])
                relfile=f'ppt/slides/_rels/slide{num}.xml.rels'
                relroot=etree.fromstring(z.read(relfile));rels={r.get('Id'):r for r in relroot}
                slide_media={}
                for rid,r in rels.items():
                    if r.get('TargetMode')=='External' and ('video' in r.get('Type','') or 'media' in r.get('Type','')):
                        issues.append(f'{stem} {num}: external-only movie')
                    if r.get('TargetMode')!='External':
                        target=posixpath.normpath(posixpath.join('ppt/slides',r.get('Target')))
                        if target not in z.namelist():issues.append(f'{stem} {num}: missing relation target {target}')
                        elif target.startswith('ppt/media/'):slide_media[rid]=sha(z.read(target))
                for im in record['images']:
                    images+=1
                    if im['sha256'] not in slide_media.values():issues.append(f'{stem} {num}: original image missing {im["source"]}')
                    shapes=[s for s in sl.shapes if hasattr(s,'image') and sha(s.image.blob)==im['sha256']]
                    if len(shapes)!=1 or any(getattr(shapes[0],'crop_'+k,0) for k in ('left','right','top','bottom')):
                        issues.append(f'{stem} {num}: cropped image')
                xml=etree.fromstring(z.read(f'ppt/slides/slide{num}.xml'))
                for run in xml.xpath('.//a:r',namespaces=NS):
                    rpr=run.find('a:rPr',NS)
                    if rpr is None or int(rpr.get('sz','0'))<2000:issues.append(f'{stem} {num}: text smaller than 20 pt')
                    fills=[value.upper() for value in rpr.xpath('./a:solidFill/a:srgbClr/@val',namespaces=NS)] if rpr is not None else []
                    if fills not in [['142D38'],['006D68']]:issues.append(f'{stem} {num}: unexpected/gray text color {fills}')
                for video in record['videos']:
                    videos+=1
                    if video['sha256'] not in slide_media.values():issues.append(f'{stem} {num}: missing embedded video')
                    if video['autoplay']:autoplay+=1
                if sum(v['autoplay'] for v in record['videos'])!=len(xml.xpath('.//p:video/p:cMediaNode/p:cTn/p:stCondLst/p:cond[@delay="0"]',namespaces=NS)):
                    issues.append(f'{stem} {num}: autoplay count')
                movie_digests=[]
                for el in xml.xpath('.//a:videoFile',namespaces=NS):
                    movie_digests.append(slide_media.get(el.get('{'+NS['r']+'}link')))
                if sorted(movie_digests)!=sorted(v['sha256'] for v in record['videos']):issues.append(f'{stem} {num}: video relationship mismatch')
        if manifest.get('source_code_url'):
            cover_links=[run.hyperlink.address for shape in prs.slides[0].shapes if shape.has_text_frame
                         for paragraph in shape.text_frame.paragraphs for run in paragraph.runs]
            if manifest['source_code_url'] not in cover_links:issues.append(f'{stem}: missing cover source hyperlink')
        introduced=[entry for entry in bounds if entry not in inherited_bounds]
        if introduced:issues.append({stem+'_out_of_bounds':introduced})
        if inherited_bounds:warnings.append({'unchanged_user_layout_out_of_bounds':inherited_bounds,'scope':'The same complete shape XML exists in the pre-update snapshot. Text is checked in the PDF; the user layout was preserved.'})
        if notes!=len(pages):issues.append(f'{stem}: missing notes')
        pdf=OUT/(stem+'.pdf')
        info=subprocess.run(['pdfinfo',str(pdf)],capture_output=True,text=True,check=True).stdout
        pdfpages=int(re.search(r'^Pages:\s+(\d+)',info,re.M).group(1))
        if pdfpages!=len(pages):issues.append(f'{stem}: PDF page count')
        raw=subprocess.run(['pdftotext','-layout',str(pdf),'-'],capture_output=True,text=True,check=True).stdout
        page_text=raw.split('\f')[:len(pages)];missing=[]
        for sl,record,pdftext in zip(prs.slides,pages,page_text):
            if norm(record['title']) not in norm(pdftext):issues.append(f'{stem} {record["page"]}: title lost in PDF')
            for content in texts(sl):
                # Extracted text can interleave two columns; verify each original paragraph separately.
                for line in content.split('\n'):
                    if line.strip() and norm(line) not in norm(pdftext):missing.append({'page':record['page'],'text':line})
        if missing:issues.append({stem+'_pdf_text_check':missing})
        if '�' in raw:issues.append(f'{stem}: replacement glyph')
        if args.render:render(pdf,OUT/'preview')
        decks.append({'file':file.name,'slides':len(pages),'pdf_pages':pdfpages,'stage_page_ranges':{f'L{i}':[r['page'] for r in pages if r['stage']==f'L{i}'] for i in range(5)},'original_image_placements':images,'embedded_video_placements':videos,'autoplay_movies':autoplay,'editable_tables':tables,'source_notes':notes,'out_of_bounds':bounds,'pptx_sha256':sha(file.read_bytes()),'pdf_sha256':sha(pdf.read_bytes())})
    for path,digest in manifest['source_sha256'].items():
        if sha((ROOT/path).read_bytes())!=digest:issues.append('Source changed: '+path)
    if len(manifest['decks'])!=1 or len(list(OUT.glob('*.pptx')))!=1:issues.append('Only one formal PPTX is allowed')
    if len(list(OUT.rglob('*.pptx')))!=1:issues.append('Old PPTX remains in subdirectory')
    full=next(iter(manifest['decks'].values()))
    used={im['source'] for r in full for im in r['images']}
    if not set(manifest['frameworks_required'])<=used:issues.append('Missing framework')
    expected=['Pro2/output/final/assets/项目验收演示_L0_L2_L4.mp4','Pro2/output/L0/walk_10s_video/demo.mp4']
    if {v['source'] for r in full for v in r['videos']}!=set(expected):issues.append('Required demos missing')
    result={'status':'pass' if not issues else 'fail','decks':decks,'source_files_checked':len(manifest['source_sha256']),'original_frameworks_reused':len(manifest['frameworks_required']),'original_report_images_reused':len(used),'minimum_native_text_pt':20,'native_text_colors':['142D38','006D68'],'checks':['one formal 15-page deck','old decks deleted','all native text >=20pt','no gray native text','PPTX ZIP CRC','chapter order','original image bytes and no cropping','selected original frameworks','MP4 internal relationships and original-byte hashes','native media timing','source notes','shape bounds','no raw LaTeX','PDF page counts and text','source file SHA256'],'issues':issues,'playback_scope':'Embedded H.264 files were fully decoded by video verification; PPTX media and timing were checked structurally. PowerPoint GUI playback was not tested. PDF includes static poster frames.'}
    result['warnings']=warnings
    (OUT/'presentation_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if issues:raise SystemExit(1)

if __name__=='__main__':main()
