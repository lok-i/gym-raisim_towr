//
// Created by Jemin Hwangbo on 10/15/10.
// MIT License
//
// Copyright (c) 2010-2010 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"

raisim::World world;
auto ground = world.addGround();
//uncoment this line if u want run it here
//but for the folder struct of gym env
// the later line

auto anymal = world.addArticulatedSystem(raisim::loadResource("anymal/anymal.urdf"));

//auto anymal = world.addArticulatedSystem("./rsc/anymal/anymal.urdf");
float base_initial_height_def = 0.54;     
     

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

extern "C"{
void _init_ViSsetup(bool gravity)
{


 if(!gravity)
 world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
 world.setTimeStep(0.0025);

  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(2600, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setKeyboardCallback(raisimKeyboardCallback);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);
  vis->setDesiredFPS(25);

  //simulation is automatically stepped, if is false
  raisim::gui::manualStepping = true; 
  //raisim::gui::Collisionbodies = true; 
  /// starts visualizer thread
  vis->initApp();

  
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
  auto anymalVis = vis->createGraphicalObject(anymal, "anymal");

  vis->select(groundVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);



}
}

extern "C"{
void _init_anymal(float base_initial_height = base_initial_height_def)
{

  anymal->setGeneralizedCoordinate({0, 0, base_initial_height, 1.0, 0.0, 0.0, 0.0, 

-0.13535572325766684, 0.9743811894650634, -1.332805236697449, 
 0.1353557332372563, 0.9743812194285772, -1.3328052761745393, 
-0.13535573056942238, -0.9743812218456691, 1.3328052791030047, 
 0.13535572529984888, -0.9743811832125653, 1.3328052297175719});


Eigen::VectorXd jointPgain(anymal->getDOF()), jointDgain(anymal->getDOF()),jointVelocityTarget(anymal->getDOF());
jointPgain.setZero();
jointDgain.setZero();
jointVelocityTarget.setZero();
  
 // P and D gains for the leg actuators alone
jointPgain.tail(12).setConstant(200.0);
jointDgain.tail(12).setConstant(10.0);

  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setName("anymal");
  

  anymal->printOutBodyNamesInOrder();
}}

extern "C"{
void _rst(float base_initial_height = base_initial_height_def)
{

 anymal->setGeneralizedCoordinate({0, 0, base_initial_height, 1.0, 0.0, 0.0, 0.0, 

-0.13535572325766684, 0.9743811894650634, -1.332805236697449, 
 0.1353557332372563, 0.9743812194285772, -1.3328052761745393, 
-0.13535573056942238, -0.9743812218456691, 1.3328052791030047, 
 0.13535572529984888, -0.9743811832125653, 1.3328052297175719});
  // world.integrate1();
  // world.integrate2();
}}

void controller_1(float angles[12])
{//to take random samples
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.7);
  std::srand(std::time(nullptr));


 

    static size_t controlDecimation = 0;


    if (controlDecimation % 50 != 0)
    return;

    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(anymal->getDOF()+1), jointVelocityTarget(anymal->getDOF());
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0, // doesnt matter as the pd gains are zero
                          0, 0, 0, 0,// 
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0;

        

        for (size_t k = 0; k < anymal->getGeneralizedCoordinateDim() ; k++)
        {
        
        if(k>=7)
         jointNominalConfig(k) = angles[k-7];


         }
       
         //std::cout<<anymal->getGeneralizedCoordinateDim();
      
        anymal->setPdTarget(jointNominalConfig, jointVelocityTarget);
}






extern "C"{
  
void _close()

{
  auto vis = raisim::OgreVis::get();
  vis->closeApp();
  _rst();
}

}


extern "C"{


void get_state(float* state)

{

// raisim::Vec<3> point_W;
// anymal->getFramePosition(0,point_W);
// std::cout<<"\n\n"<<"Root frame position\n";

// for(int i=0;i<3;i++)
//   std::cout<<point_W[i]<<"\t";

// raisim::Mat<3, 3> orientation_W;
// anymal->getFrameOrientation(0,orientation_W);
// std::cout<<"\n\n"<<"Root frame orientation\n";
// for(int i =0;i<3;i++)
//   {std::cout<<"\n";
//    for(int j=0;j<3;j++)
//       std::cout<<orientation_W(i,j)<<"\t";

//   }



raisim::VecDyn GenCo(anymal->getGeneralizedCoordinate());
raisim::VecDyn GenW(anymal->getGeneralizedVelocity());
raisim::VecDyn GenT(anymal->getGeneralizedForce());


/*
std::cout<<"\n\n"<<"GenCo\n"<<GenCo;
std::cout<<"\n\n"<<"GenW\n"<<GenW;
std::cout<<"\n\n"<<"GenT\n"<<GenT;
std::cout<<"\n\n";
*/

int len = anymal->getGeneralizedCoordinateDim();

for (int i =0;i<len;i++)
*(state+i) = GenCo.v[i];

for (int i =0;i<len-7;i++)
*(state+i+len) = GenW.v[i+6];


for (int i =0;i<len-7;i++)
*(state+i+2*len -7) = GenT.v[i+6];



}


}



extern "C"
{
void _render()
{
auto vis = raisim::OgreVis::get();
vis->renderOneFrame();

}





}




extern "C"
{
void _sim(float angles[12],bool render = true) 
{

  double time = 0;
  double visTime = 1.0/25; // 1/fps

  auto vis = raisim::OgreVis::get();

          if(render)
          {
            if (time > -visTime * .1)
              time -= visTime;
            
            while (time < 0 ) 
            {
                controller_1(angles);
                world.integrate1();
                world.integrate2();
                time += world.getTimeStep();

                }

              /// compute how much sim is ahead


              /// do actual rendering
            
              vis->renderOneFrame();
            
          }
          else
          {

          controller_1(angles);
          world.integrate1();
          world.integrate2();
                



          }
}}

