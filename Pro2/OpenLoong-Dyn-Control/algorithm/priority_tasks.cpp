/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "priority_tasks.h"
#include <algorithm>
#include <limits>

namespace {

// Pseudoinverse for the symmetric positive-semidefinite task metric J P J^T.
// Forming the metric through several numerical null-space projectors can leave
// tiny asymmetric or negative eigenvalues.  Symmetrizing and discarding those
// near-zero directions prevents an infeasible task direction from being
// amplified into a large acceleration correction.
Eigen::MatrixXd symmetricPseudoInverse(const Eigen::MatrixXd &matrix)
{
    const Eigen::MatrixXd symmetric = 0.5 * (matrix + matrix.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
    if (solver.info() != Eigen::Success)
    {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(
            symmetric);
        decomposition.setThreshold(1e-8);
        return decomposition.pseudoInverse();
    }

    const Eigen::VectorXd eigenvalues = solver.eigenvalues();
    const double largest = eigenvalues.cwiseAbs().maxCoeff();
    Eigen::VectorXd inverse = Eigen::VectorXd::Zero(eigenvalues.size());
    if (largest > 0.0)
    {
        const double relativeThreshold = std::max(
            1e-8,
            std::numeric_limits<double>::epsilon() *
                static_cast<double>(std::max(matrix.rows(), matrix.cols())));
        const double threshold = relativeThreshold * largest;
        for (Eigen::Index index = 0; index < eigenvalues.size(); ++index)
            if (eigenvalues(index) > threshold)
                inverse(index) = 1.0 / eigenvalues(index);
    }
    return solver.eigenvectors() * inverse.asDiagonal() *
           solver.eigenvectors().transpose();
}

} // namespace

void PriorityTasks::addTask(const char* name) {
    taskLib.emplace_back(name);
    taskLib.back().id=(int)(taskLib.size())-1;
    nameList.emplace_back(name);
}

int PriorityTasks::getId(const std::string &name) {
    for (size_t i = 0; i < nameList.size(); ++i) {
        if (nameList[i]==name) {
            return i;
        }
    }
    return -1;
}

int PriorityTasks::getId(const char* name){
    for (size_t i = 0; i < nameList.size(); ++i) {
        if (nameList[i]==name) {
            return i;
        }
    }
    printf("Cannot find wbc task: %s !\n", name);
    return -1;
}

void PriorityTasks::buildPriority(const std::vector<std::string> &taskOrder) {
    startId= getId(taskOrder[0]);
    for (int i=0;i<taskOrder.size();i++)
    {
        int idCur= getId(taskOrder[i]);
        if (i==0)
            taskLib[idCur].parentId=-1;
        else
        {
            int idBef= getId(taskOrder[i-1]);
            taskLib[idCur].parentId=idBef;
        }
        if (i==taskOrder.size()-1)
            taskLib[idCur].childId=-1;
        else
        {
            int idNxt= getId(taskOrder[i+1]);
            taskLib[idCur].childId=idNxt;
        }
    }
}

void PriorityTasks::printTaskInfo() {
    for (int i=0;i<taskLib.size();i++)
    {
        printf("-------------\n");
        printf("taskName=%s\n",taskLib[i].taskName.c_str());
        printf("parentId=%d, childId=%d\n",taskLib[i].parentId,taskLib[i].childId);
    }
}

void PriorityTasks::computeAll(const Eigen::VectorXd &des_delta_q,const Eigen::VectorXd &des_dq, const Eigen::VectorXd &des_ddq, const Eigen::MatrixXd &dyn_M, const Eigen::MatrixXd &dyn_M_inv, const Eigen::VectorXd &dq, bool strictHierarchy) {
    int curId=startId;
    int parentId=taskLib[curId].parentId;
    int childId=taskLib[curId].childId;
    for (int i=0;i<taskLib.size();i++)
    {
        if (parentId==-1){
            taskLib[curId].N=Eigen::MatrixXd::Identity(taskLib[curId].J.cols(),taskLib[curId].J.cols());
            taskLib[curId].Jpre=taskLib[curId].J*taskLib[curId].N;
            taskLib[curId].delta_q=des_delta_q+ pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W)*taskLib[curId].errX;
            taskLib[curId].dq=des_dq;
            Eigen::VectorXd ddxcmd= taskLib[curId].ddxDes + taskLib[curId].kp * taskLib[curId].errX+taskLib[curId].kd*taskLib[curId].derrX;
            taskLib[curId].ddq= des_ddq + dyn_pseudoInv(taskLib[curId].Jpre,dyn_M_inv,true) * (ddxcmd - taskLib[curId].dJ * dq);
//            std::cout<<taskLib[curId].taskName<<std::endl<<taskLib[curId].delta_q.transpose()<<std::endl;
        }
        else{
            taskLib[curId].N=taskLib[parentId].N*
                    (Eigen::MatrixXd::Identity(taskLib[parentId].Jpre.cols(),taskLib[parentId].Jpre.cols())- pseudoInv_right_weighted(taskLib[parentId].Jpre,taskLib[parentId].W)*taskLib[parentId].Jpre);
            taskLib[curId].Jpre=taskLib[curId].J*taskLib[curId].N;
            taskLib[curId].delta_q=taskLib[parentId].delta_q+ pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W)*(taskLib[curId].errX-
                    taskLib[curId].J*taskLib[parentId].delta_q);
            taskLib[curId].dq=taskLib[parentId].dq+ pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W)*(taskLib[curId].dxDes-
                    taskLib[curId].J*taskLib[parentId].dq);
            Eigen::VectorXd ddxcmd= taskLib[curId].ddxDes + taskLib[curId].kp * taskLib[curId].errX+taskLib[curId].kd*taskLib[curId].derrX;
            const Eigen::VectorXd ddqResidual =
                ddxcmd-taskLib[curId].dJ*dq-taskLib[curId].J*taskLib[parentId].ddq;
            if (strictHierarchy)
            {
                // Restrict every lower-priority acceleration correction to
                // the null space accumulated from all preceding tasks.  The
                // old dyn_pseudoInv(J*N) expression did not explicitly
                // left-project the correction and could therefore disturb a
                // higher-priority fixed-contact task near singular poses.
                Eigen::MatrixXd projectedMinv =
                    taskLib[curId].N * dyn_M_inv * taskLib[curId].N.transpose();
                projectedMinv = 0.5 * (projectedMinv + projectedMinv.transpose());
                Eigen::MatrixXd taskMetric =
                    taskLib[curId].J * projectedMinv * taskLib[curId].J.transpose();
                const Eigen::MatrixXd strictJbar =
                    projectedMinv * taskLib[curId].J.transpose() *
                    symmetricPseudoInverse(taskMetric);
                taskLib[curId].ddq =
                    taskLib[parentId].ddq + strictJbar * ddqResidual;
            }
            else
            {
                taskLib[curId].ddq= taskLib[parentId].ddq + dyn_pseudoInv(taskLib[curId].Jpre,dyn_M_inv,true) * ddqResidual;
            }
//            std::cout<<taskLib[curId].taskName<<std::endl<<taskLib[curId].delta_q.transpose()<<std::endl;
        }
//        printf("task: %s\n", taskLib[curId].taskName.c_str());
//        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(taskLib[curId].Jpre);
//        printf("taskJacobian rank: %d, rows: %d\n", lu_decomp.rank(), taskLib[curId].Jpre.rows());
        if (childId!=-1){
            parentId=curId;
            curId=childId;
            childId=taskLib[curId].childId;
        }
        else
            break;
    }
    out_delta_q=taskLib[curId].delta_q;
    out_dq=taskLib[curId].dq;
    out_ddq=taskLib[curId].ddq;
}













