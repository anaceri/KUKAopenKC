/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Kuka Starting Test
 ============================================================================
 */


#include <iostream>
#include <thread>
#include <unistd.h>

#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"
#include "kukaselfctrltask.h"
#include "tacservocontroller.h"
#include "tacservotask.h"
//#include "CtrlParam.h"


ComOkc *com_okc;
Robot *kuka_lwr;
ActController *ac;
Task *task;
TaskNameT taskname;
ParameterManager* pm;

#define newP_x 0.28
#define newP_y 0.5
#define newP_z 0.50

#define newO_x 0.0
#define newO_y 0;
#define newO_z 0.0;


char inp;

RobotModeT rmt;

//The function we want to make the thread run.
void keypresscap(void)
{	char inp_tmp;
    while(inp != 'e'){
        inp_tmp = getchar();
            if ( inp_tmp != '\n' ) {
            inp = inp_tmp;
            std::cout<<"in sub task loop, you input char "<<inp<<std::endl;
            }
    }
}

void moveto_cb(void){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x-0.3;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    delete ac;
    delete task;
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt == JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    rmt = NormalMode;
    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void psudog_cb(void){
    rmt = PsudoGravityCompensation;
}

void print_pf(void){
    Eigen::Vector3d p,f,t;
    Eigen::Matrix3d o;

    p = kuka_lwr->get_cur_cart_p();
    o = kuka_lwr->get_cur_cart_o();
    kuka_lwr->get_eef_ft(f,t);
    std::cout<<"position "<<p[0]<<","<<p[1]<<","<<p[2]<<std::endl;
    std::cout<<"orientation "<<std::endl;std::cout<<o<<std::endl;
    std::cout<<"force "<<f[0]<<","<<f[1]<<","<<f[2]<<std::endl;
    std::cout<<"torque "<<t[0]<<","<<t[1]<<","<<t[2]<<std::endl;
}

int counter = 0;

void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
//        kuka_lwr->update_robot_stiffness();
        kuka_lwr->get_joint_position_act();
        kuka_lwr->update_robot_state();
        //using all kinds of controllers to update the reference
        if(task->mt == JOINTS)
            ac->update_robot_reference(kuka_lwr,task);
        ac->llv.setZero();
        ac->lov.setZero();
        //use CBF to compute the desired joint angle rate
        kuka_lwr->update_cbf_controller();
        kuka_lwr->set_joint_command(rmt);
        com_okc->controller_update = true;
        counter++;
        if(counter >=25){
            print_pf();
            counter = 0;
        }
    }
}


void init(){
    pm = new ParameterManager("right_arm_param.xml");
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_lwr = new KukaLwr(kuka_right,*com_okc);
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    kuka_lwr->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           ac->pm.stiff_ctrlpara.axis_damping);
    rmt = NormalMode;
}

int main(int argc, char* argv[])
{
    std::thread t1(keypresscap);
    inp = 'f';
    init();
    while(inp != 'e'){
        switch (inp){
        case 'm':
            moveto_cb();
            inp = '\n';
            break;
        case 'g':
            psudog_cb();
            inp = '\n';
            break;
        case '\n':
            break;
        default:
            break;
        }
        run();
        usleep(20);
}
    std::cout<<"main function is end "<<std::endl;
    t1.join();
    std::cout<<"keypress thread is end "<<std::endl;
}

