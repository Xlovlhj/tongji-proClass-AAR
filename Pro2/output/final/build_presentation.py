"""Build the single 15-page formal presentation from existing stage evidence."""
from pathlib import Path
import hashlib,json,re
from PIL import Image
from pptx import Presentation
from pptx.util import Inches,Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.xmlchemy import OxmlElement

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
STEM='青龙MPC_项目汇报_15页'
FONT='Noto Sans CJK SC';INK='142D38';TEAL='006D68';PALE='E9F4F2';WHITE='FFFFFF'
DOC='Pro2/机器人智能控制_作业2_足式机器人MPC开源实践.docx'
REPORTS={i:f'Pro2/output/L{i}/'+n for i,n in enumerate(['L0验收报告.md','L1源码分析报告.md','L2验收报告.md','L3理论联系报告.md','L4验收报告.md'])}
L0='Pro2/output/L0/';L1='Pro2/output/L1/';L2='Pro2/output/L2/';L3='Pro2/output/L3/';L4='Pro2/output/L4/'
H1=L4+'01_robot_swap/h1_fixstand_feedback_8s_final_v2/analysis/'
PATH=L4+'02_path_tracking/';TERR=L4+'03_terrain_and_jump/'
JUMP=TERR+'vertical_jump_acceptance_run01/analysis_final/'
MPC=L4+'04_mpc_comparison/analysis_same_binary/'
TOR=L4+'05_control_decomposition/azureloong_walk_same_binary/figures/'
FRAMEWORKS=[L1+'figures/L1闭环控制框架.png',L3+'figures/L3理论关系图.png',L4+'figures/L4凸MPC与NMPC对比.png',L4+'figures/L4力矩分解链路.png']
PAGES=[];SOURCES=set()


def color(s):return RGBColor.from_string(s)
def source(path):
    p=ROOT/path
    if not p.is_file():raise FileNotFoundError(p)
    SOURCES.add(path)
    if path not in PAGES[-1]['sources']:PAGES[-1]['sources'].append(path)
    return p

def text(sl,x,y,w,h,value,size=22,bold=False,accent=False):
    assert size>=20, 'No small print'
    sh=sl.shapes.add_textbox(Inches(x),Inches(y),Inches(w),Inches(h))
    tf=sh.text_frame;tf.clear();tf.word_wrap=True
    tf.margin_left=tf.margin_right=tf.margin_top=tf.margin_bottom=0
    for i,line in enumerate(value.split('\n')):
        p=tf.paragraphs[0] if i==0 else tf.add_paragraph()
        p.space_after=Pt(5);p.line_spacing=1.08
        r=p.add_run();r.text=line;r.font.name=FONT;r.font.size=Pt(size)
        r.font.bold=bold;r.font.color.rgb=color(TEAL if accent else INK)
        rp=r._r.get_or_add_rPr();rp.set('lang','zh-CN')
        ea=OxmlElement('a:ea');ea.set('typeface',FONT);rp.append(ea)
    return sh

def panel(sl,x,y,w,h):
    sh=sl.shapes.add_shape(MSO_SHAPE.RECTANGLE,Inches(x),Inches(y),Inches(w),Inches(h))
    sh.fill.solid();sh.fill.fore_color.rgb=color(PALE);sh.line.fill.background()
    for e in sh._element.xpath('./p:style/a:effectRef'):e.set('idx','0')
    return sh

def table(sl,x,y,w,h,rows,widths=None,size=21):
    sh=sl.shapes.add_table(len(rows),len(rows[0]),Inches(x),Inches(y),Inches(w),Inches(h));tb=sh.table
    if widths:
        for c,v in zip(tb.columns,widths):c.width=Inches(w*v)
    for i,row in enumerate(rows):
        tb.rows[i].height=Inches(h/len(rows))
        for j,v in enumerate(row):
            c=tb.cell(i,j);c.text=str(v);c.vertical_anchor=MSO_ANCHOR.MIDDLE
            c.margin_left=c.margin_right=Inches(.10);c.margin_top=c.margin_bottom=Inches(.025)
            c.fill.solid();c.fill.fore_color.rgb=color(PALE if i==0 else WHITE)
            for p in c.text_frame.paragraphs:
                p.space_after=Pt(0);p.line_spacing=1.02
                for r in p.runs:
                    r.font.name=FONT;r.font.size=Pt(size);r.font.bold=i==0;r.font.color.rgb=color(INK)
                    ea=OxmlElement('a:ea');ea.set('typeface',FONT);r._r.get_or_add_rPr().append(ea)
    return sh

def image(sl,path,x,y,w,h):
    p=source(path)
    with Image.open(p) as im:iw,ih=im.size
    k=min(w/iw,h/ih)
    sh=sl.shapes.add_picture(str(p),Inches(x+(w-iw*k)/2),Inches(y+(h-ih*k)/2),Inches(iw*k),Inches(ih*k))
    sh.name='原图 '+path
    PAGES[-1]['images'].append({'source':path,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
    return sh

def movie(sl,path,poster,x,y,w,h):
    p=source(path);pp=source(poster)
    sh=sl.shapes.add_movie(str(p),Inches(x),Inches(y),Inches(w),Inches(h),poster_frame_image=str(pp),mime_type='video/mp4')
    sh.name='内嵌视频 '+path
    PAGES[-1]['videos'].append({'source':path,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'poster':poster,'embedded':True,'autoplay':False})

def slide(prs,stage,title,sections=(),notes=''):
    sl=prs.slides.add_slide(prs.slide_layouts[6]);sl.background.fill.solid();sl.background.fill.fore_color.rgb=color(WHITE)
    label='项目汇报' if stage is None else f'L{stage}'
    PAGES.append({'page':len(PAGES)+1,'stage':label,'title':title,'report':REPORTS.get(stage),'sections':list(sections),'notes':notes,'sources':[],'images':[],'videos':[]})
    if stage is not None:source(REPORTS[stage])
    text(sl,.55,.38,12.2,.66,(f'L{stage}  ' if stage is not None else '')+title,32,True)
    return sl

def experiment(prs,title,img,section,condition,rows,conclusion,reason,config):
    sl=slide(prs,2,title,[section])
    text(sl,.6,1.22,12.1,.52,condition,22)
    image(sl,img,.55,1.98,7.45,3.95)
    table(sl,8.3,2.0,4.42,2.25,rows,size=21)
    text(sl,8.35,4.48,4.3,1.35,conclusion,22,True,True)
    text(sl,.65,6.0,12,.78,reason,21)
    text(sl,.65,6.95,12,.4,config,20)
    return sl


def build():
    p=Presentation();p.slide_width=Inches(13.333333);p.slide_height=Inches(7.5)
    p.core_properties.title='青龙人形机器人 MPC 开源实践'
    p.core_properties.subject='15页正式汇报：L0、L1、L2、L3、L4'
    # 1: a single cover, containing the required complete acceptance demonstration.
    s=slide(p,None,'青龙人形机器人 MPC 开源实践')
    source(DOC)
    text(s,.65,1.35,12,.55,'OpenLoong 控制栈 · Isaac Sim 仿真适配',28,True,True)
    movie(s,'Pro2/output/final/assets/项目验收演示_L0_L2_L4.mp4','Pro2/output/final/assets/poster.png',.55,2.08,7.75,4.359375)
    text(s,8.65,2.18,4.05,2.95,'L0  跑通与调参\nL1  控制原理与源码\nL2  四组对照实验\nL3  理论联系\nL4  五项挑战',24)
    text(s,.7,6.63,7.55,.5,'2 分 40 秒：跑通、四组对照与综合演示',22,True,True)
    text(s,8.65,5.63,4.05,1.5,'保留开源 MPC / WBC\n完成 Isaac 接口与分析\n工程实现使用 AI 辅助',22)
    # 2-3: L0, completely consecutive.
    s=slide(p,0,'环境搭建与默认运行',[1,2,3,4,5,7])
    table(s,.6,1.3,7.25,2.72,[['实际环境','配置'],['系统','Ubuntu 22.04.5 双系统'],['仿真','Isaac Sim 6.0.1 / PhysX'],['编译','g++ 11.4 / CMake 3.22'],['原路线','人形 MuJoCo，无需 ROS']],[.26,.74],20)
    movie(s,L0+'walk_10s_video/demo.mp4',L0+'walk_10s_video/final_frame.png',8.2,1.42,4.55,3.4125)
    text(s,.7,4.22,7.1,.92,'关节映射、基座速度转换、力矩接口完成适配；关闭 Isaac drive，避免重复 PD。',21)
    text(s,8.25,5.05,4.4,1.0,'默认 10 s：净位移 1.172 m\nMPC / WBC 非零状态均为 0',21,True,True)
    text(s,.65,5.36,7.4,.38,'从项目根目录检查、构建、启动：',22,True,True)
    text(s,.7,5.96,12.0,1.25,'python3 Pro2/run_isaac.py --check-env\npython3 Pro2/run_isaac.py --build-only\npython3 Pro2/run_isaac.py --level L0 --duration 10 --walk-start 3',20)
    s=slide(p,0,'输入代价权重：实际调参与分析',[6,8,9])
    text(s,.65,1.25,12,.52,'u_weight：10⁻⁷ / 10⁻⁶（默认）/ 10⁻⁵；每次 12 s，统计 5–11.99 s。',22)
    image(s,L0+'mpc_input_weight/comparison/speed_tracking.png',.55,2.03,7.6,3.92)
    table(s,8.45,2.05,4.25,2.25,[['输入权重','速度 RMSE'],['10⁻⁷','0.02189 m/s'],['10⁻⁶','0.02361 m/s'],['10⁻⁵','0.02021 m/s']],size=21)
    text(s,8.5,4.65,4.15,1.35,'10⁻⁵ 相对默认\n速度误差 −14.39%\n控制努力 +2.83%',23,True,True)
    text(s,.65,6.06,12,.56,'增大接触输入代价改变了优化折中；10⁻⁵ 是当前场景三候选中的推荐。',22)
    text(s,.65,6.8,12,.4,'配置：Pro2/experiments/L0/mpc_input_weight/high_1e-5.json',20)
    # 4-5: L1.
    s=slide(p,1,'MPC / WBC 闭环与优化问题',[3,4,5,7])
    image(s,FRAMEWORKS[0],.5,1.26,4.25,5.91)
    text(s,5.1,1.4,7.55,.45,'MPC 规划未来接触力，WBC 协调全身运动。',24,True,True)
    text(s,5.1,2.17,7.55,1.05,'状态：姿态、位置、角速度、线速度，共 12 维。\n输入：左右脚力与力矩 + 固定重力增广量，共 13 维。',21)
    text(s,5.1,3.5,7.55,1.43,'模型：在线线性化单刚体。\n目标：状态误差与接触输入偏差的二次代价。\n约束：摩擦、单边接触、足底力矩与支撑切换。',21)
    text(s,5.1,5.15,7.55,1.75,'求解：qpOASES；只执行第一步接触输入。\n预测 10 × 5 ms = 50 ms；独立控制 3 步。\nMPC 200 Hz；WBC / PVT / 物理 1 kHz。\n当前状态来自 Isaac 真值。',21)
    s=slide(p,1,'源码定位与关节力矩执行',[6,7,8,9,10,11])
    text(s,.65,1.28,12,.49,'控制目录：Pro2/OpenLoong-Dyn-Control/；表内 algorithm 与 common 均相对此处。',20)
    code_table=table(s,.6,1.98,12.1,3.86,[['模块','源码 / 函数'],['状态反馈（适配层）','Pro2/isaac/simulation.py::_run()'],['步态 / 足端规划','algorithm/gait_scheduler.cpp::step()\nalgorithm/foot_placement.cpp::getSwingPos()'],['MPC：预测与优化','algorithm/mpc.cpp::cal() / set_weight()'],['WBC：全身运动','algorithm/wbc_priority.cpp::computeDdq() / computeTau()'],['PVT：反馈与限幅','common/PVT_ctrl.cpp\ncommon/joint_ctrl_config.json']],[.29,.71],20)
    for row,height in zip(code_table.table.rows,[.48,.5,.94,.5,.5,.94]):row.height=Inches(height)
    text(s,.7,6.06,12,1.1,'WBC：分层伪逆 / 零空间求运动目标，再用动力学 QP 得到前馈力矩。\nPVT：滤波 PD + WBC 前馈 → maxTorque 限幅 → Isaac 关节侧力矩。',22)
    # 6-9: all four comparisons, one page each.
    experiment(p,'对照实验一：前向速度权重',L2+'mpc_vx_weight/comparison/speed_tracking.png',4,
      'state_weights[9]：0.2 / 1.0（默认）/ 5.0；速度统计窗口 5–11.99 s。',
      [['速度权重','RMSE (m/s)'],['0.2','0.04430'],['1.0','0.02354'],['5.0','0.01814']],
      '高权重相对默认\n误差 −22.9%\n峰值力矩 +11.2%',
      '更重视速度误差，跟踪更紧；封面对照为当前库补录，曲线为原正式运行。',
      '配置：Pro2/experiments/L2/mpc_vx_weight/high.json')
    experiment(p,'对照实验二：预测时域',L2+'prediction_horizon/comparison/solver_time.png',5,
      'mpc_horizon_steps：5 / 10（默认）/ 15；对应 25 / 50 / 75 ms。',
      [['预测窗口','RMSE (m/s)'],['25 ms','0.02482'],['50 ms','0.02354'],['75 ms','0.02613']],
      '50 ms 跟踪最好\n75 ms 抽样控制步\n耗时增加 36.1%',
      '更长时域增加计算负担；图中为 QP CPU 时间，抽样控制步耗时另行统计。',
      '配置：Pro2/experiments/L2/prediction_horizon/long_n15.json')
    experiment(p,'对照实验三：抬腿高度',L2+'step_height/comparison/base_height.png',6,
      'step_height：0.08 / 0.12（默认）/ 0.16 m；平地，目标速度 0.2 m/s。',
      [['期望步高','RMSE (m/s)'],['0.08 m','0.01788'],['0.12 m','0.02354'],['0.16 m','0.02708']],
      '0.08 m 平地负担较小\n绝对机械功代理\n相对默认 −28.8%',
      '降低摆腿幅度减轻全身运动负担；该结果不能直接外推为越障性能更好。',
      '配置：Pro2/experiments/L2/step_height/low_008.json')
    push=json.loads((ROOT/L2/'lateral_push/long_push_20261105/comparison/presentation_summary.json').read_text())
    push_text=push['ppt']
    experiment(p,'对照实验四：侧向扰动',push['figure'],7,
      push_text['condition'],push_text['table_rows'],push_text['conclusion'],push_text['reason'],
      '配置：'+push_text['config_path'])
    # 10: all three theoretical questions.
    s=slide(p,3,'LQR、iLQR 与 MPC 的理论联系',[2,3,4,5,6])
    image(s,FRAMEWORKS[1],.5,1.3,4.15,5.82)
    text(s,5.0,1.4,7.7,.48,'方法与执行框架：分别回答怎样求解、怎样执行。',23,True,True)
    text(s,5.0,2.15,7.7,1.65,'LQR：线性二次问题，Riccati 求反馈增益。\niLQR：局部线性化，后向求增益、前向修正轨迹。\nMPC：按当前状态重算未来，只执行第一步。',22)
    text(s,5.0,4.08,7.7,1.1,'固定增益的局限：接触切换、摩擦与单边约束、\n模型变化和力矩饱和，需要显式约束与前瞻。',22)
    text(s,5.0,5.53,7.7,1.5,'凸 MPC：计算较轻，受局部模型近似限制。\nNMPC：保留非线性，计算与收敛负担更大。\n本阶段为理论分析，未运行 LQR / iLQR。',22)
    # 11-15: five independent L4 challenges.
    s=slide(p,4,'挑战一：更换 H1 模型并站立',[3])
    text(s,.65,1.26,12,.5,'真实 H1-2 Handless USD，27 关节；3 s 过渡 + 5 s 保持。',22)
    image(s,H1+'base_height.png',.55,2.02,7.45,3.97)
    table(s,8.3,2.03,4.4,2.9,[['验收指标','实测值'],['最低高度','0.9893 m'],['最大单轴倾角','0.07764 rad'],['水平最大漂移','0.06073 m'],['最终关节 RMSE','0.03650 rad']],size=20)
    text(s,.7,6.12,12,1.0,'独立 FixStand PD + 骨盆反馈站立通过。\nOpenLoong MPC / WBC 尚未迁移到 H1。',23,True,True)
    s=slide(p,4,'挑战二：给定路径跟踪',[4])
    text(s,.65,1.25,12,.55,'路径误差生成 vx、vy、wz，经限幅送入原 MPC / WBC 行走闭环。',22)
    image(s,PATH+'circle_closed_loop_run02/analysis/planar_trajectory.png',.55,2.0,6.45,4.0)
    table(s,7.3,2.02,5.4,3.52,[['路径','RMSE (m)','到点'],['直线','0.07909','是'],['90° 转弯','0.20102','是'],['圆开环','0.14480','否'],['圆闭环','0.13395','是'],['多路径点','0.21732','锁存']],[.4,.35,.25],20)
    text(s,.7,6.11,12,1.04,'五组完成，四组触发到点。圆开 / 闭环时长和容差不同，不能直接比较优劣。\n多路径点最终距目标 0.2874 m：到点事件与最终停稳位置分别评价。',21)
    s=slide(p,4,'挑战三：复杂地形与竖直跳跃',[5])
    image(s,JUMP+'height.png',.55,1.57,7.45,4.27)
    text(s,8.35,1.52,4.3,.44,'地形结果',25,True,True)
    text(s,8.35,2.25,4.3,1.36,'随机障碍：区域穿越。\n坡道、粗糙地面、盲踩：\n仅完成部分进入。',22)
    text(s,8.35,3.95,4.3,.44,'跳跃结果',25,True,True)
    text(s,8.35,4.68,4.3,1.36,'两次各 21 项验收通过。\n日志增高 0.0608 m。\n腾空持续 0.269 s。',22)
    text(s,.7,6.22,12,.94,'腾空时关闭接触 MPC，着陆后恢复双足支撑。\n实际高度未达到 0.12 m 目标，落地接触冲击仍需优化。',23,True)
    s=slide(p,4,'挑战四：凸 MPC 与 NMPC 对照',[6])
    image(s,FRAMEWORKS[2],.5,1.3,3.4,5.8)
    table(s,4.25,1.55,8.45,3.16,[['同控制库 10 s 对照','凸 MPC','SQP-RTI NMPC'],['前向 RMSE (m/s)','0.03532','0.03408'],['平均求解 (ms)','0.355','17.129'],['求解失败','采样未见','36 / 350 更新']],[.43,.25,.32],21)
    text(s,4.3,5.0,8.3,1.98,'前向误差接近，当前 NMPC 实现计算更重。\nNMPC 失败 / 接触不匹配时回退到 C++ 输入。\n误差取全时段，耗时取行走期；控制周期也不同。\n各一次运行，不能据此推断普遍性能优劣。',22)
    s=slide(p,4,'挑战五：足底力与关节力矩分解',[7])
    image(s,FRAMEWORKS[3],.45,1.3,3.16,5.8)
    image(s,TOR+'mpc_vertical_contact_force.png',3.85,1.6,4.3,3.6)
    image(s,TOR+'J_ankle_l_pitch.png',8.42,1.6,4.3,3.6)
    text(s,3.9,5.46,8.8,1.55,'WBC 前馈 + 滤波 PD → 限幅 → Isaac 施加。\n左 / 右踝限幅 5 / 10 点；命令回读差约 10⁻⁶ N·m。\n学习结论：调参需同时看跟踪、稳定性与控制代价。',21)
    assert len(p.slides)==15
    # Keep exact evidence locations in notes and the delivery source index.
    notes=['# 逐页讲稿与证据来源\n','正文使用深色正常字号；本文件保存来源与讲解顺序。\n']
    for sl,rec in zip(p.slides,PAGES):
        body='\n'.join(sh.text for sh in sl.shapes if sh.has_text_frame)
        rec['body_text']=body
        sources='\n'.join(rec['sources'])
        sl.notes_slide.notes_text_frame.text=body+'\n\n证据来源（项目根相对路径）：\n'+sources
        notes += [f"## 第 {rec['page']} 页｜{rec['stage']}｜{rec['title']}\n",body+'\n','来源：\n'+'\n'.join('- `'+v+'`' for v in rec['sources'])+'\n']
    # Analytical images must already be in one of the five original reports.
    allowed=set()
    for report in REPORTS.values():
        rp=ROOT/report
        for rel in re.findall(r'!\[[^\]]*\]\(([^)]+)\)',rp.read_text()):
            allowed.add(str((rp.parent/rel).resolve().relative_to(ROOT)))
    used={im['source'] for r in PAGES for im in r['images']}
    assert used<=allowed,used-allowed
    assert set(FRAMEWORKS)<=used
    p.save(OUT/(STEM+'.pptx'))
    manifest={'revision':'one formal 15-page deck; no gray small print; no footers','decks':{STEM:PAGES},'frameworks_required':FRAMEWORKS,'original_report_image_count':len(used),'minimum_text_pt':20,'allowed_text_colors':[INK,TEAL],'source_sha256':{v:hashlib.sha256((ROOT/v).read_bytes()).hexdigest() for v in sorted(SOURCES)}}
    (OUT/'presentation_sources.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (OUT/'逐页讲稿与证据来源.md').write_text('\n'.join(notes))
    print(json.dumps({'pages':15,'file':STEM+'.pptx','original_images':len(used),'original_frameworks':len(FRAMEWORKS),'embedded_movies':sum(len(r['videos']) for r in PAGES)},ensure_ascii=False))

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--update-cover-only',action='store_true',help='Preserve the current user-edited PPT and replace only its cover video')
    args=parser.parse_args()
    if args.update_cover_only:
        from update_cover_video import main
        main()
    else:
        build()
