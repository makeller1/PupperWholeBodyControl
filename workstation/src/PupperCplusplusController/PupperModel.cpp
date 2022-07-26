#include <iostream>
#include "workstation/PupperModel.h"

// #define DEBUG_MODE // Uncomment to print debug information

// RBDL Model Notes: 
// 1.  COM of an added/joined body is described in child frame (the frame of the body which is being added to another body).
// 2.  The inertia of the child is described in the child's frame at its COM.
// 3.  The tranformation used when joining/adding bodies has a rotation and translation. The translation is relative to the parent's original unrotated frame.
// 4.  Inertias are described in the COM despite the contradictory documentation of RBDL.

using std::cout;
using std::endl;

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

#define FIXED Joint(JointTypeFixed)
#define SQUARE(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

namespace {
    // const double inch2meter = 0.0254;
    const double mm2meter   = 0.001;
    const double gram2kg = 0.001;

    // Convert mass moment of inertia from g*mm^2 to kg*m^2
    Matrix3d scaleMMOI(Matrix3d M){
        return M * 1e-9;
    }

    // Inertia matrix of a homogenous cuboid measured at the center of mass
    Matrix3d cubeInertia(double x_dim, double y_dim, double z_dim, double mass){
        double ixx = 1/12.0 * mass * (SQUARE(y_dim) + SQUARE(z_dim));
        double iyy = 1/12.0 * mass * (SQUARE(x_dim) + SQUARE(z_dim));
        double izz = 1/12.0 * mass * (SQUARE(x_dim) + SQUARE(y_dim));
        Matrix3d inertia;
        inertia << ixx, 0,   0,
                   0,   iyy, 0,
                   0,   0,   izz;
        return inertia;
    }

    // Inertia matrix of a uniform cylinder measured at the center of mass
    Matrix3d cylinderInertia(double radius, double height, double mass){
        // Define the axis of the cylinder in the z-direction
        double ihh = 1/12.0 * mass * (3 * SQUARE(radius) + SQUARE(height));
        double irr = 1/2.0 * mass * SQUARE(radius);
        Matrix3d inertia;
        inertia << ihh, 0,   0,
                   0,   ihh, 0,
                   0,   0,   irr;
        return inertia;
    }

}

void printCOM(RigidBodyDynamics::Model& model){
    static int k = 0;
    Vector3d com_location;
    double model_mass;
    com_location.setZero();
    VectorNd joint_angles_(model.q_size);
    VectorNd joint_velocities_(model.qdot_size);
    joint_angles_.setZero();
    joint_angles_(model.q_size-1) = 1.0;
    RigidBodyDynamics::Utils::CalcCenterOfMass(model,joint_angles_,joint_velocities_,NULL, model_mass, com_location);
    cout << "Count " << k << ")" << endl;
    RigidBodyDynamics::Math::MatrixNd massMat;
    massMat.resize(model.q_size,model.q_size);
    massMat.setZero(); // Required!
    CompositeRigidBodyAlgorithm(model, joint_angles_, massMat, false);
    // cout << "M: \n" << massMat_ << endl;
    cout << "Mass: " << massMat(0,0) << endl;
    cout << "Center of mass location: " << com_location.transpose() << endl;
    cout << "\n" << endl;
    k+=1;
}

std::shared_ptr<Model> createPupperModel(){
    
    auto model = std::make_shared<Model>();
    model->gravity = Vector3d(0, 0, -9.81);
    
    // ============================================================================= //
    //                       Measured / Estimated Properties                         //
    // ============================================================================= //
    bool in_simulation = true; // If simulating - model rotor inertia inaccurately
    double rotor_inertia = 0.0028 * 1e9; // g*mm^2 Estimated reflected rotor inertia

    // ============================================================================= //
    //                         Properties from SolidWorks                            //
    // ============================================================================= //
    // According to spreadsheet "Mass Properties and Irigin Transforms" in pupper_description
    // Note: all link frames have the same orientation

    /* --- Bottom PCB with all fixed bodies --- */
    double bottom_pcb_mass = 1086.5; //g
    Vector3d bottom_pcb_COM = Vector3d(-16.46, 0, 37.53); // mm
    Matrix3d bottom_pcb_MOI; // g*mm^2
    bottom_pcb_MOI << 1919398.64, 235.71, 1884.89, 
                        235.71, 8759021.12, 10.99,
                        1884.89, 10.99, 9498546.46;
    Vector3d bottom_pcb_translation = Vector3d(0,0,0); // (mm) translation from the root frame to the origin of the bottom_PCB frame

    /* --- Back Left Hub --- */
    double back_left_hub_mass = 109.8; //g
    Vector3d back_left_hub_COM = Vector3d(-29,  0.31, 0); // mm
    Matrix3d back_left_hub_MOI; // g*mm^2
    back_left_hub_MOI << 23576.71, -34.26, -0.01,
                        -34.26, 12321.79, 0.01,
                        -0.01,  0.01, 27265.99;
    Vector3d back_left_hub_translation = Vector3d(-122.00, 47, 43.5); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Back Right Hub --- */
    double back_right_hub_mass = 109.8; //g
    Vector3d back_right_hub_COM = Vector3d(-29, -0.31, 0); // mm
    Matrix3d back_right_hub_MOI; // g*mm^2
    back_right_hub_MOI << 23576.71, -34.26, -0.01,
                        -34.26, 12321.79, -0.02,
                        -0.01,  -0.02, 27265.99;
    Vector3d back_right_hub_translation = Vector3d(-122, -47, 43.5); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Front Left Hub --- */
    Vector3d front_left_hub_translation = Vector3d(78, 47, 43.5); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Front Right Hub --- */
    Vector3d front_right_hub_translation = Vector3d(78, -47, 43.5); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Left Upper Link --- */
    double back_left_upper_link_mass = 143.55; // g
    Vector3d back_left_upper_link_COM = Vector3d(0, 36.76, -59.83); // mm
    Matrix3d back_left_upper_link_MOI; // g*mm^2
    back_left_upper_link_MOI << 171730.23,  0.01, -0.06,
                                0.01, 141191.83, 45763.54,
                                -0.06, 45763.54, 43033.83;
    Vector3d back_left_upper_link_translation = Vector3d(-31.50, 21.5, 0); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Right Upper Link --- */
    double back_right_upper_link_mass = 143.55; // g
    Vector3d back_right_upper_link_COM = Vector3d(0, -36.76, -59.83); // mm
    Matrix3d back_right_upper_link_MOI; // g*mm^2
    back_right_upper_link_MOI << 171730.23, 0.01, 0.06,
                                0.01, 141191.83, -45763.54,
                                0.06, -45763.54, 43033.83;
    Vector3d back_right_upper_link_translation = Vector3d(-31.50, -21.5, 0); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Left Lower Link --- */
    double back_left_lower_link_mass = 21; // g
    Vector3d back_left_lower_link_COM = Vector3d(-0.22, -12.19, -33.35); // mm
    Matrix3d back_left_lower_link_MOI; // g*mm^2
    back_left_lower_link_MOI << 43753.77, 12.76, -359.76,
                                12.76, 44138.15, 456.47,
                                -359.76, 456.47, 856.89;
    Vector3d back_left_lower_link_translation = Vector3d(0,23,-80); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Right Lower Link --- */
    double back_right_lower_link_mass = 21; // g
    Vector3d back_right_lower_link_COM = Vector3d(-0.22, 12.19, -33.35); // mm
    Matrix3d back_right_lower_link_MOI; // g*mm^2
    back_right_lower_link_MOI << 43753.05, -12.74, -360.13,
                                -12.74, 44137.54, -455.13,
                                -360.13, -455.13, 856.82;
    Vector3d back_right_lower_link_translation = Vector3d(0,-23,-80); // (mm) translation from the parent frame to the origin of the child frame

    /* --- Feet Locations --- */
    // Location of center of foot sphere in lower_link frame
    Vector3d back_left_foot_offset   = Vector3d(0, -9.5, -110); // mm
    Vector3d back_right_foot_offset  = Vector3d(0,  9.5, -110); // mm
    Vector3d front_left_foot_offset  = Vector3d(0, -9.5, -110); // mm
    Vector3d front_right_foot_offset = Vector3d(0,  9.5, -110); // mm

    // ============================================================================= //
    //                             Duplicate Properties                              //
    // ============================================================================= //
    double front_left_hub_mass  = back_left_hub_mass; // g
    Vector3d front_left_hub_COM = back_left_hub_COM; // mm
    Matrix3d front_left_hub_MOI = back_left_hub_MOI; // g*mm^2

    double front_right_hub_mass  = back_right_hub_mass; // g
    Vector3d front_right_hub_COM = back_right_hub_COM; // mm
    Matrix3d front_right_hub_MOI = back_right_hub_MOI; // g*mm^2

    double front_left_upper_link_mass          = back_left_upper_link_mass; // g
    Vector3d front_left_upper_link_COM         = back_left_upper_link_COM; // mm
    Matrix3d front_left_upper_link_MOI         = back_left_upper_link_MOI; // g*mm^2
    Vector3d front_left_upper_link_translation = back_left_upper_link_translation; // (mm) translation from the parent frame to the origin of the child frame

    double   front_right_upper_link_mass        = back_right_upper_link_mass; // g
    Vector3d front_right_upper_link_COM         = back_right_upper_link_COM; // mm
    Matrix3d front_right_upper_link_MOI         = back_right_upper_link_MOI; // g*mm^2
    Vector3d front_right_upper_link_translation = back_right_upper_link_translation; // (mm) translation from the parent frame to the origin of the child frame

    double   front_left_lower_link_mass        = back_left_lower_link_mass; // g
    Vector3d front_left_lower_link_COM         = back_left_lower_link_COM; // mm
    Matrix3d front_left_lower_link_MOI         = back_left_lower_link_MOI; // g*mm^2
    Vector3d front_left_lower_link_translation = back_left_lower_link_translation; // (mm) translation from the parent frame to the origin of the child frame

    double   front_right_lower_link_mass        = back_right_lower_link_mass; // g
    Vector3d front_right_lower_link_COM         = back_right_lower_link_COM; // mm
    Matrix3d front_right_lower_link_MOI         = back_right_lower_link_MOI; // g*mm^2
    Vector3d front_right_lower_link_translation = back_right_lower_link_translation; // (mm) translation from the parent frame to the origin of the child frame

    if (in_simulation){
        Matrix3d rotor_inertia_link;
        Matrix3d rotor_inertia_hub;
        rotor_inertia_link << 0, 0, 0,
                              0, rotor_inertia, 0,
                              0, 0, 0;
        rotor_inertia_hub << rotor_inertia, 0, 0,
                             0, 0, 0,
                             0, 0, 0;
        
        // Add inertia to lower link and hip joints - This is not correct but it
        // is the only way to approximate the rotor inertia in Gazebo.
        // On hardware, the rotor inertia is added to the final mass matrix.
        back_left_hub_MOI = back_left_hub_MOI + rotor_inertia_hub;
        back_right_hub_MOI = back_right_hub_MOI + rotor_inertia_hub;
        front_left_hub_MOI = front_left_hub_MOI + rotor_inertia_hub;
        front_right_hub_MOI = front_right_hub_MOI + rotor_inertia_hub;

        back_left_lower_link_MOI = back_left_lower_link_MOI + rotor_inertia_link;
        front_left_lower_link_MOI = front_left_lower_link_MOI + rotor_inertia_link;
        back_right_lower_link_MOI = back_right_lower_link_MOI + rotor_inertia_link;
        front_right_lower_link_MOI = front_right_lower_link_MOI + rotor_inertia_link;
                                       
    }

    // ============================================================================= //
    //                                   BASE LINK                                   //
    // ============================================================================= //


    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////

    /* ------ BOTTOM PCB ------ */

    // Create the bottom PCB with all fixed components
    Body bottom_PCB = Body(bottom_pcb_mass * gram2kg, 
                           bottom_pcb_COM * mm2meter, 
                           scaleMMOI(bottom_pcb_MOI));

    /* ------ BATTERY (m) ------ */
    // // Create battery
    // double batt_mass = 0.205; // kg
    // double batt_x    = 0.035; // m
    // double batt_y    = 0.07; // m
    // double batt_z    = 0.0485; // m
    // Vector3d batt_com(0, 0, 0);
    // Matrix3d batt_inertia = cubeInertia(batt_x, batt_y, batt_z, batt_mass);
    // Body battery(batt_mass, batt_com, batt_inertia);

    // // Fixed joint connecting battery to PCB
    // SpatialTransform battery_T;
    // battery_T.E = getRotation(0, 0, 0);
    // battery_T.r = Vector3d(-.037, 0.0, batt_z/2);
    // bottom_PCB.Join(battery_T, battery);


    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////

    // Joint between PCB and ROOT
    Joint floating_joint(JointTypeFloatingBase);
    uint bottom_PCB_id = model->AddBody(0, Xtrans(bottom_pcb_translation * mm2meter), floating_joint, bottom_PCB, "bottom_PCB");

    // ============================================================================= //
    //                                   LEGS                                        //
    // ============================================================================= //
    

    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////

    /* ------ FOOT (VIRTUAL) ------ */
    // Convenience body with zero mass/inertia used to retrieve location of foot. The mass/inertia is already lumped with lower link body
    Body foot = Body(0.0, Vector3dZero, Matrix3dZero); 



    /* ------ BACK LEFT HUB ------ */
    Body back_left_hub = Body(back_left_hub_mass * gram2kg, 
                              back_left_hub_COM * mm2meter, 
                    scaleMMOI(back_left_hub_MOI));

    /* ------ BACK RIGHT HUB ------ */
    Body back_right_hub = Body(back_right_hub_mass * gram2kg, 
                               back_right_hub_COM * mm2meter, 
                     scaleMMOI(back_right_hub_MOI));
    
    /* ------ FRONT LEFT HUB ------ */
    Body front_left_hub = Body(front_left_hub_mass * gram2kg, 
                               front_left_hub_COM * mm2meter, 
                     scaleMMOI(front_left_hub_MOI));

    /* ------ FRONT RIGHT HUB ------ */
    Body front_right_hub = Body(front_right_hub_mass * gram2kg, 
                                front_right_hub_COM * mm2meter, 
                      scaleMMOI(front_right_hub_MOI));



    /* ------ BACK LEFT UPPER LINK ------ */
    Body back_left_upper_link = Body(back_left_upper_link_mass * gram2kg, 
                                     back_left_upper_link_COM * mm2meter, 
                           scaleMMOI(back_left_upper_link_MOI));

    /* ------ BACK RIGHT UPPER LINK ------ */
    Body back_right_upper_link = Body(back_right_upper_link_mass * gram2kg, 
                                      back_right_upper_link_COM * mm2meter, 
                            scaleMMOI(back_right_upper_link_MOI));

    /* ------ FRONT LEFT UPPER LINK ------ */
    Body front_left_upper_link = Body(front_left_upper_link_mass * gram2kg, 
                                      front_left_upper_link_COM * mm2meter, 
                            scaleMMOI(front_left_upper_link_MOI));

    /* ------ FRONT RIGHT UPPER LINK ------ */
    Body front_right_upper_link = Body(front_right_upper_link_mass * gram2kg, 
                                       front_right_upper_link_COM * mm2meter, 
                             scaleMMOI(front_right_upper_link_MOI));



    /* ------ BACK LEFT LOWER LINK ------ */
    Body back_left_lower_link = Body(back_left_lower_link_mass * gram2kg, 
                                     back_left_lower_link_COM * mm2meter, 
                           scaleMMOI(back_left_lower_link_MOI));

    /* ------ BACK RIGHT LOWER LINK ------ */
    Body back_right_lower_link = Body(back_right_lower_link_mass * gram2kg, 
                                      back_right_lower_link_COM * mm2meter, 
                            scaleMMOI(back_right_lower_link_MOI));

    /* ------ FRONT LEFT LOWER LINK ------ */
    Body front_left_lower_link = Body(front_left_lower_link_mass * gram2kg, 
                                      front_left_lower_link_COM * mm2meter, 
                            scaleMMOI(front_left_lower_link_MOI));

    /* ------ FRONT RIGHT LOWER LINK ------ */
    Body front_right_lower_link = Body(front_right_lower_link_mass * gram2kg, 
                                       front_right_lower_link_COM * mm2meter, 
                             scaleMMOI(front_right_lower_link_MOI));



    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////
    
    /* ---------- BACK LEFT LEG ---------- */

    // Revolute joint connecting back left hub to PCB
    Joint back_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_left_hub_T;
    back_left_hub_T.E = Matrix3dIdentity;
    back_left_hub_T.r = back_left_hub_translation * mm2meter;
    uint back_left_hub_id = model->AddBody(bottom_PCB_id, back_left_hub_T, back_left_hub_joint, back_left_hub, "back_left_hub");

    // Revolute joint connecting back left upper link to back left hub
    Joint back_left_shoulder_joint(JointTypeRevolute, Vector3d(0, 1, 0));
    SpatialTransform back_left_shoulder_T;
    back_left_shoulder_T.E = Matrix3dIdentity;
    back_left_shoulder_T.r = back_left_upper_link_translation * mm2meter;
    uint back_left_upper_link_id = model->AddBody(back_left_hub_id, back_left_shoulder_T, back_left_shoulder_joint, back_left_upper_link, "back_left_upper_link");

    // Revolute joint connecting back left lower link to back left upper link
    Joint back_left_elbow_joint(JointTypeRevolute, Vector3d(0, -1, 0));
    SpatialTransform back_left_elbow_T;
    back_left_elbow_T.E = Matrix3dIdentity;
    back_left_elbow_T.r = back_left_lower_link_translation * mm2meter;
    uint back_left_lower_link_id = model->AddBody(back_left_upper_link_id, back_left_elbow_T, back_left_elbow_joint, back_left_lower_link, "back_left_lower_link");


    /* ---------- BACK RIGHT LEG ---------- */

    // Revolute joint connecting back right hub to PCB
    Joint back_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_right_hip_T;
    back_right_hip_T.E = Matrix3dIdentity;
    back_right_hip_T.r = back_right_hub_translation * mm2meter;
    uint back_right_hub_id = model->AddBody(bottom_PCB_id, back_right_hip_T, back_right_hip_joint, back_right_hub, "back_right_hub");

    // Revolute joint connecting back right upper link to back right hub
    Joint back_right_shoulder_joint(JointTypeRevolute, Vector3d(0, -1, 0));
    SpatialTransform back_right_shoulder_T;
    back_right_shoulder_T.E = Matrix3dIdentity;
    back_right_shoulder_T.r = back_right_upper_link_translation * mm2meter;
    uint back_right_upper_link_id = model->AddBody(back_right_hub_id, back_right_shoulder_T, back_right_shoulder_joint, back_right_upper_link, "back_right_upper_link");

    // Revolute joint connecting back right lower link to back right upper link
    Joint back_right_elbow_joint(JointTypeRevolute, Vector3d(0, 1, 0));
    SpatialTransform back_right_elbow_T;
    back_right_elbow_T.E = Matrix3dIdentity;
    back_right_elbow_T.r = back_right_lower_link_translation * mm2meter;
    uint back_right_lower_link_id = model->AddBody(back_right_upper_link_id, back_right_elbow_T, back_right_elbow_joint, back_right_lower_link, "back_right_lower_link");


    /* ---------- FRONT LEFT LEG ---------- */

    // Revolute joint connecting front left hub to PCB (FL HIP)
    Joint front_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_left_hub_T;
    front_left_hub_T.E = Matrix3dIdentity;
    front_left_hub_T.r = front_left_hub_translation * mm2meter;
    uint front_left_hub_id = model->AddBody(bottom_PCB_id, front_left_hub_T, front_left_hub_joint, front_left_hub, "front_left_hub");             

    // Revolute joint connecting front left upper link to front left hub (FL SHOULDER)
    Joint front_left_shoulder_joint(JointTypeRevolute, Vector3d(0, 1, 0));
    SpatialTransform front_left_shoulder_T;
    front_left_shoulder_T.E = Matrix3dIdentity;   
    front_left_shoulder_T.r = front_left_upper_link_translation * mm2meter;       
    uint front_left_upper_link_id = model->AddBody(front_left_hub_id, front_left_shoulder_T, front_left_shoulder_joint, front_left_upper_link, "front_left_upper_link");

    // Revolute joint connecting front left upper link and front left lower link (FL ELBOW)
    Joint front_left_elbow_joint = Joint(JointTypeRevolute, Vector3d(0, -1, 0));
    SpatialTransform front_left_elbow_T;
    front_left_elbow_T.E = Matrix3dIdentity;
    front_left_elbow_T.r = front_left_lower_link_translation * mm2meter;
    uint front_left_lower_link_id = model->AddBody(front_left_upper_link_id, front_left_elbow_T, front_left_elbow_joint, front_left_lower_link, "front_left_lower_link");


    /* ---------- FRONT RIGHT LEG ---------- */

    // Revolute joint connecting front right hub to PCB (FR HIP)
    Joint front_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_right_hub_T;
    front_right_hub_T.E = Matrix3dIdentity;
    front_right_hub_T.r = front_right_hub_translation * mm2meter;
    uint front_right_hub_id = model->AddBody(bottom_PCB_id, front_right_hub_T, front_right_hip_joint, front_right_hub, "front_right_hub");

    // Revolute joint connecting front right upper link to front right hub (FR SHOULDER)
    Joint front_right_shoulder_joint(JointTypeRevolute, Vector3d(0, -1, 0));
    SpatialTransform front_right_upper_link_T;
    front_right_upper_link_T.E = Matrix3dIdentity;
    front_right_upper_link_T.r = front_right_upper_link_translation * mm2meter;
    uint front_right_upper_link_id = model->AddBody(front_right_hub_id, front_right_upper_link_T, front_right_shoulder_joint, front_right_upper_link, "front_right_upper_link");

    // Revolute joint connection front right lower link to front right upper link (FR ELBOW)
    Joint front_right_elbow_joint(JointTypeRevolute, Vector3d(0, 1, 0));
    SpatialTransform front_right_elbow_T;
    front_right_elbow_T.E = Matrix3dIdentity;
    front_right_elbow_T.r = front_right_lower_link_translation * mm2meter;
    uint front_right_lower_link_id = model->AddBody(front_right_upper_link_id, front_right_elbow_T, front_right_elbow_joint, front_right_lower_link, "front_right_lower_link");


    /* ---------- FEET ---------- */

    // Fixed joint connecting back left foot to back left lower link
    SpatialTransform back_left_foot_T;
    back_left_foot_T.E = Matrix3dIdentity;
    back_left_foot_T.r = back_left_foot_offset * mm2meter;
    uint back_left_foot_id = model->AddBody(back_left_lower_link_id, back_left_foot_T, FIXED, foot, "back_left_foot");

    // Fixed joint connecting back right foot to back right lower link
    SpatialTransform back_right_foot_T;
    back_right_foot_T.E = Matrix3dIdentity;
    back_right_foot_T.r = back_right_foot_offset * mm2meter;
    uint back_right_foot_id = model->AddBody(back_right_lower_link_id, back_right_foot_T, FIXED, foot, "back_right_foot");

    // Fixed joint connecting front left foot to front left lower link
    SpatialTransform front_left_foot_T;
    front_left_foot_T.E = Matrix3dIdentity;
    front_left_foot_T.r = front_left_foot_offset * mm2meter;
    uint front_left_foot_id = model->AddBody(front_left_lower_link_id, front_left_foot_T, FIXED, foot, "front_left_foot");

    // Fixed joint connecting front right foot to front right lower link
    SpatialTransform front_right_foot_T;
    front_right_foot_T.E = Matrix3dIdentity;
    front_right_foot_T.r = front_right_foot_offset * mm2meter;
    uint front_right_foot_id = model->AddBody(front_right_lower_link_id, front_right_foot_T, FIXED, foot, "front_right_foot");

    
    #ifdef DEBUG_MODE
        cout << "Bottom PCB inertial properties: " << endl;
        cout << "Mass: " << bottom_PCB.mMass << endl;
        cout << "Inertia: \n" << bottom_PCB.mInertia << endl;
        cout << "\nFeet positions in neutral config: " << endl;
        VectorNd joint_angles_(model->q_size);
        joint_angles_.setZero();
        joint_angles_(model->q_size-1) = 1.0;
        std::vector<const char*> points_to_test = {"front_left_foot", "front_right_foot", "back_left_foot", "back_right_foot"};
        for (auto body : points_to_test){
            auto pos = CalcBodyToBaseCoordinates(*model, joint_angles_, model->GetBodyId(body), {0,0,0});
            cout << "Location of " << body << " in base coordinates: (" << pos[0] << ", " << pos[1] << ", "  << pos[2] << ")\n";
        }

        // printCOM(*model);
        
        // Test to reveal getRotation conventions and RBDL transformation conventions. 
        // cout << "TEST OF getRotation (90, 0.0, 0.0) : " << endl;
        // cout << getRotation(M_PI_2, 0.0, 0.0) << endl << endl;

        // cout << "TEST OF getRotation (90, 90, 0.0) : " << endl;
        // cout << getRotation(M_PI_2, M_PI_2, 0.0) << endl << endl;

        // cout << "TEST OF getRotation (90, 90, 90) : " << endl;
        // cout << getRotation(M_PI_2, M_PI_2, M_PI_2) << endl << endl;

        // // Test of RBDL transformation
        // Vector3d base_com = Vector3d(0.0,0.0,0.0);
        // Matrix3d base_inertia;
        // double base_mass = 0.0;
        // base_inertia << 0, 0,         0, 
        //                0,        0, 0,
        //                0,        0,         0;
        // Body base_body_test = Body(base_mass, base_com, base_inertia);

        // Vector3d new_body_com = Vector3d(0.0,0.0,-5.0);
        // Matrix3d new_body_inertia;
        // double new_body_mass = 1.0;
        // new_body_inertia << 0, 0,         0, 
        //                0,        0, 0,
        //                0,        0,         0;
        // Body new_body = Body(new_body_mass, new_body_com, new_body_inertia);

        // // Join new body to base body
        // SpatialTransform base_to_new;
        // base_to_new.r = Vector3d(0, -5.0, 0);
        // base_to_new.E = getRotation(M_PI_2, 0, 0);
        // base_body_test.Join(base_to_new, new_body);
        // cout << "Test of RBDL transformation: " << endl;
        // cout << "Below should read (0.0, 0.0, 0.0)" << endl;
        // cout << "New COM of base: " << base_body_test.mCenterOfMass.transpose() << endl;

        // Test of RBDL Join/Add body function
        // Inertia is described in COM.
        // Vector3d base_com = Vector3d(0.0,0.0,0.0);
        // Matrix3d base_inertia;
        // double base_mass = 0.1;
        // base_inertia << 0.1, 0, 0, 
        //                 0, 0.1, 0,
        //                 0, 0, 0.1;
        // Body base_body_test = Body(base_mass, base_com, base_inertia);

        // Vector3d new_body_com = Vector3d(-10.0,-10.0,-10.0);
        // Matrix3d new_body_inertia;
        // double new_body_mass = 10.0;
        // new_body_inertia << 1, 0, 0, 
        //                     0, 1, 0,
        //                     0, 0, 1;
        // Body new_body = Body(new_body_mass, new_body_com, new_body_inertia);

        // // Join new body to base body
        // SpatialTransform base_to_new;
        // base_to_new.r = Vector3d(10.0, 10.0, 10.0);
        // base_to_new.E = getRotation(0, 0, 0);
        // base_body_test.Join(base_to_new, new_body);
        // cout << "Test of RBDL join: " << endl;
        // cout << "If the inertia of body is in the COM, then below should read diag (1.1, 1.1, 1.1)" << endl;
        // cout << "Combined Inertia: \n" << base_body_test.mInertia << endl;

    #endif
    

    return model;
}