#include <iostream>
#include "workstation/PupperModel.h"

// RBDL Model Notes: 
// 1.  COM of an added/joined body is described in child frame (the frame of the body which is being added to another body).
// 2.  The inertia of the child is described in the child's frame at its COM.
// 3.  The tranformation used when joining/adding bodies has a rotation and translation. The translation is relative to the parent's original unrotated frame.
// 4.  Inertias are described in the COM despite the contradictory documentation of RBDL.

using std::cout;
using std::endl;

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

// #define DEBUG_MODE // Uncomment to print debug information
#define FIXED Joint(JointTypeFixed)

#define SQUARE(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

namespace {
    const double inch2meter = 0.0254;
    const double mm2meter   = 0.001;
    const double density    = 0.3 * 1240; // kg/m^3 with infill of 30%

    Matrix3d scaleInteria(Matrix3d M, double rho = density){
        // Meshlab interia is in mm^2 * mm^3
        // To get the actual inertia in kg*m^2 we have to 
        // multiply by 0.001 ^ 5 * density
        return M * density * pow(mm2meter, 5);
    }

    Matrix3d scaleInteriaInches(Matrix3d M, double rho = density){
        // Meshlab interia is in in^2 * in^3
        // To get the actual inertia in kg*m^2 we have to 
        // multiply by (0.0254) ^ 5 * density
        return M * density * pow(inch2meter, 5);
    }

    Matrix3d getRotation(double x, double y, double z){
        // Order of rotations: Rotate about X, then Y, then Z.
        // Rotations are about the original frame axes (The axes of rotations remain fixed). 
        Quaternion q;
        q = q.fromXYZAngles(Vector3d(x, y, z));
        return q.toMatrix();
    }

    // Inertia matrix of a uniform cuboid measured at the center of mass
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
    
    // Z direction is up
    model->gravity = Vector3d(0, 0, -9.81);

    // Calculate the densities of some of the materials 
    // These numbers come from experimental data of parts that we weighed

    /* --- Bottom PCB --- */

    double pcb_volume = 4.038224 * pow(inch2meter, 3);         // in^3 -> m^3
    double bottom_pcb_mass = 0.088 + 0.035;                    // kg (adding estimated weight of solder + left/right shrouds)
    double bottom_pcb_density = bottom_pcb_mass/pcb_volume;    // kg/m^3

    /* --- Top PCB --- */

    double top_pcb_mass = 0.056;
    double top_pcb_density = top_pcb_mass/pcb_volume;

    /* --- Thin PLA --- */

    const double thin_PLA_mass = 0.032;
    const double thin_PLA_volume = 28243.179688 * CUBE(mm2meter);
    const double thin_PLA_density = thin_PLA_mass/thin_PLA_volume;

    /* --- Thick PLA --- */

    const double thick_PLA_mass = 0.007;
    const double thick_PLA_volume = 13170.648438 * CUBE(mm2meter);
    const double thick_PLA_density = thick_PLA_mass/thick_PLA_volume;

    /* --- Battery location in x of bottom_pcb frame --- */
    double battery_loc_x = -.17; // -.17 from -.028

    /* --- Tuned additional inertia of bottom PCB--- */
    double tuned_Iyy = 0.0; 

    // ============================================================================= //
    //                                   BASE LINK                                   //
    // ============================================================================= //


    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////

    /* ------ BOTTOM PCB (inches) ------ */

    // Create the bottom PCB
    Vector3d pcb_com = inch2meter * Vector3d(-0.134500, -0.000013, -0.000000);
    Matrix3d pcb_inertia;
    pcb_inertia << 4.405234, 0,         0, 
                   0,        41.052544 + tuned_Iyy, 0,
                   0,        0,         45.448391;
    pcb_inertia = scaleInteriaInches(pcb_inertia, bottom_pcb_density);
    Body bottom_PCB = Body(bottom_pcb_mass, pcb_com, pcb_inertia);

    /* ------ TOP PCB (inches) ------ */

    // Create the top PCB
    pcb_inertia = scaleInteriaInches(pcb_inertia, top_pcb_density);
    Body top_PCB = Body(top_pcb_mass, pcb_com, pcb_inertia);

    /* ------ FRONT BULKHEAD (mm) ------ */

    double front_bulkhead_mass = 28243.179688 * pow(mm2meter,3) * thin_PLA_density;
    Vector3d front_bulkhead_com = mm2meter * Vector3d(0.0, 0.0, 245.537643);
    Matrix3d front_bulkhead_inertia;
    front_bulkhead_inertia << 27491730.000000,  -18.677107,        -1.684238, 
                             -18.677107,         74592784.000000,  -1.507474,
                              -1.684238,        -1.507474,          83928296.000000;
    front_bulkhead_inertia = scaleInteria(front_bulkhead_inertia, thin_PLA_density);
    Body front_bulkhead(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);
    #ifdef DEBUG_MODE
        cout << "front_bulkhead_mass (kg): " << front_bulkhead_mass << endl;
    #endif

    /* ------ MIDDLE BULKHEAD 1 (mm) and MOTOR CONTROLLER (m) ------ */

    double middle_bh_1_mass = 33243.015625 * pow(mm2meter,3) * thin_PLA_density;
    Vector3d middle_bh_1_com = mm2meter * Vector3d(0.050467, -0.533781, 202.751328);
    Matrix3d middle_bh_1_inertia;
    middle_bh_1_inertia << 22996456.000000,  -192.887802,      -639.832642,
                          -192.887802,        51008240.000000, -22193.009766,
                          -639.832642,       -22193.009766,     73639128.000000;
    middle_bh_1_inertia = scaleInteria(middle_bh_1_inertia, thin_PLA_density);
    Body middle_bh_1(middle_bh_1_mass, middle_bh_1_com, middle_bh_1_inertia);

    // -------------------------------------

    double motor_controller_mass = 0.022;
    Vector3d motor_controller_com = Vector3d(0, 0, 0);
    Matrix3d motor_controller_inertia = cubeInertia(0.03, 0.01, 0.003, motor_controller_mass);
    Body motor_controller(motor_controller_mass, motor_controller_com, motor_controller_inertia);

    // -------------------------------------

    SpatialTransform front_bulkhead2controller;
    // 2 on the Left side
    front_bulkhead2controller.r = middle_bh_1_com + Vector3d(0.02, 0, 0.015);
    front_bulkhead2controller.E = getRotation(0, 0, 0);
    middle_bh_1.Join(front_bulkhead2controller, motor_controller);
    middle_bh_1.Join(front_bulkhead2controller, motor_controller);
    // 2 on the Right side
    front_bulkhead2controller.r = middle_bh_1_com + Vector3d(-0.02, 0, 0.015);
    middle_bh_1.Join(front_bulkhead2controller, motor_controller);
    middle_bh_1.Join(front_bulkhead2controller, motor_controller);

    #ifdef DEBUG_MODE
        cout << "middle_bh_1_mass (kg): " << middle_bh_1.mMass << endl;
    #endif

    /* ------ MIDDLE BULKHEAD 2 (mm) ------ */

    double middle_bh_2_mass = 33262.445312 * pow(mm2meter,3) * thin_PLA_density;
    Vector3d middle_bh_2_com = mm2meter * Vector3d(-0.147800, 0.000000, 127.256111);
    Matrix3d middle_bh_2_inertia;
    middle_bh_2_inertia << 21912060.000000, -49.972462,       -10377.245117,
                          -49.972462,        40583024.000000, -0.003996,
                          -10377.245117,    -0.003996,         62043808.000000;
    middle_bh_2_inertia = scaleInteria(middle_bh_2_inertia, thin_PLA_density);
    Body middle_bh_2(middle_bh_2_mass, middle_bh_2_com, middle_bh_2_inertia);

    #ifdef DEBUG_MODE
        cout << "middle_bh_2_mass (kg): " << middle_bh_2.mMass << endl;
    #endif
    /* ------ REAR BULKHEAD (mm) and MOTOR CONTROLLERS (m) ------ */

    double rear_bh_mass = 33735.734375 * pow(mm2meter,3) * thin_PLA_density;
    Vector3d rear_bh_com = mm2meter * Vector3d(-0.004437, -0.593278, 2.799114);
    Matrix3d rear_bh_inertia;
    rear_bh_inertia << 23357426.000000, -4462.781738,      427.257507,
                      -4462.781738,      52151916.000000, -17892.154297,
                       427.257507,      -17892.154297,     75135296.000000;
    rear_bh_inertia = scaleInteria(rear_bh_inertia, thin_PLA_density);
    Body rear_bh(rear_bh_mass, rear_bh_com, rear_bh_inertia);

    // ---------------------------------------

    SpatialTransform rear_bulkhead2controller;
    // 2 on the Left side
    rear_bulkhead2controller.r = rear_bh_com + Vector3d(0.02, 0, 0.015);
    rear_bulkhead2controller.E = getRotation(0, 0, 0);
    rear_bh.Join(rear_bulkhead2controller, motor_controller);
    rear_bh.Join(rear_bulkhead2controller, motor_controller);
    // 2 on the Right side
    rear_bulkhead2controller.r = rear_bh_com + Vector3d(-0.02, 0, 0.015);
    rear_bh.Join(rear_bulkhead2controller, motor_controller);
    rear_bh.Join(rear_bulkhead2controller, motor_controller);

    #ifdef DEBUG_MODE
        cout << "rear_bh_mass (kg): " << rear_bh.mMass << endl;
    #endif

    // Left and right shroud removed - negligible contribution to inertia significant source of mistakes
    /* ------ LEFT SHROUD (mm) ------ */

    // double left_shroud_mass = 15699.697266 * pow(mm2meter,3) * thin_PLA_density;
    // Vector3d left_shroud_com = mm2meter * Vector3d(56.354546, -0.000083, 67.307045);
    // Matrix3d left_shroud_inertia;
    // left_shroud_inertia <<  30641502.000000, -101.878265,       1374721.250000,
    //                        -101.878265,       21280568.000000,  78.329567,
    //                         1374721.250000,   78.329567,        10692940.000000;
    // left_shroud_inertia = scaleInteria(left_shroud_inertia, thin_PLA_density);
    // Body left_shroud(left_shroud_mass, left_shroud_com, left_shroud_inertia);
    
    // #ifdef DEBUG_MODE
    //     cout << "left_shroud_mass (kg): " << left_shroud_mass << endl;
    // #endif
    /* ------ RIGHT SHROUD (mm) ------ */

    // double right_shroud_mass = 15699.833984 * pow(mm2meter,3) * thin_PLA_density;
    // Vector3d right_shroud_com = mm2meter * Vector3d(-56.354591, 0.000044, 67.307060);
    // Matrix3d right_shroud_inertia;
    // right_shroud_inertia << 30641556.000000, -117.941475,      -1374718.500000,
    //                        -117.941475,       21280556.000000, -71.618393,
    //                        -1374718.500000,  -71.618393,        10692993.000000;
    // right_shroud_inertia = scaleInteria(right_shroud_inertia, thin_PLA_density);
    // Body right_shroud(right_shroud_mass, right_shroud_com, right_shroud_inertia);

    // #ifdef DEBUG_MODE
    //     cout << "right_shroud_mass (kg): " << right_shroud_mass << endl;
    // #endif
    /* ------ BATTERY (m) ------ */

    double batt_mass = 0.205 + 0.06;
    double batt_x    = 0.07;
    double batt_y    = 0.035;
    double batt_z    = 0.0485;
    Vector3d batt_com(0, 0, 0);
    Matrix3d batt_inertia = cubeInertia(batt_x, batt_y, batt_z, batt_mass);
    Body battery(batt_mass, batt_com, batt_inertia);

    /* ----- IMU AND BREADBOARD (m) ----- */

    double imu_mass = 0.017; // includes breadboard
    double imu_x    = 0.04;
    double imu_y    = 0.02;
    double imu_z    = 0.005;
    Vector3d imu_com = Vector3d(0, 0, 0);
    Matrix3d imu_inertia = cubeInertia(imu_x, imu_y, imu_z, imu_mass);
    Body imu(imu_mass, imu_com, imu_inertia);

    /* ----- POWER BUTTON ----- */

    double power_mass = 0.044;
    double power_rad  = 0.01;
    double power_len  = 0.03;
    Vector3d power_com = Vector3d(0, 0, 0);
    Matrix3d power_inertia = cylinderInertia(power_rad, power_len, power_mass);
    Body power_button(power_mass, power_com, power_inertia);

    /* ------  MOTOR (m) ------ */
    double motor_rad  = 0.012;
    double motor_len  = 0.03;
    double motor_mass = 0.09;
    Vector3d motor_com = Vector3d(0, 0, 0);
    Matrix3d motor_inertia = cylinderInertia(motor_rad, motor_len, motor_mass);
    Body motor(motor_mass, motor_com, motor_inertia);

    /* ---------- LUMPED BOTTOM PCB ---------- */
    
    // Fixed joint connecting front top PCB to bottom PCB
    SpatialTransform bottom2top;
    bottom2top.E = getRotation(0, 0, 0);
    bottom2top.r = Vector3d(0, 0.00, 0.087);
    bottom_PCB.Join(bottom2top, top_PCB);

    // Fixed joint connecting front bulkhead to PCB
    SpatialTransform front_bulkhead_T;
    front_bulkhead_T.E << 0, 1, 0,
                          0, 0, 1, 
                          1, 0, 0;
    front_bulkhead_T.r = Vector3d(-0.13, 0.00, 0.042);
    bottom_PCB.Join(front_bulkhead_T, front_bulkhead);

    // Fixed joint connecting middle bulkhead to PCB
    SpatialTransform middle_bh_1_T;
    middle_bh_1_T.E = getRotation(M_PI_2, 0, M_PI_2);
    middle_bh_1_T.r = Vector3d(-0.128, 0.00, 0.044);
    bottom_PCB.Join(middle_bh_1_T, middle_bh_1);

    // Fixed joint connecting middle bulkhead 2 to PCB
    SpatialTransform middle_bh_2_T;
    middle_bh_2_T.E = getRotation(M_PI_2, 0, M_PI_2);
    middle_bh_2_T.r = Vector3d(-0.128, 0.00, 0.044);
    bottom_PCB.Join(middle_bh_2_T, middle_bh_2);

    // Fixed joint connecting rear bulkhed to PCB
    SpatialTransform rear_bh_T;
    rear_bh_T.E = getRotation(M_PI_2, 0, M_PI_2);
    rear_bh_T.r = Vector3d(-0.128, 0.00, 0.044);
    bottom_PCB.Join(rear_bh_T, rear_bh);

    // Fixed joint connecting battery to PCB
    SpatialTransform battery_T;
    battery_T.E = getRotation(0, 0, M_PI_2);
    battery_T.r = Vector3d(battery_loc_x, 0.0, 0.02425);
    bottom_PCB.Join(battery_T, battery);

    // Fixed joint connecting IMU to PCB
    SpatialTransform IMU_T;
    IMU_T.E = getRotation(0, 0, 0);
    IMU_T.r = Vector3d(0.04, 0, 0);
    bottom_PCB.Join(IMU_T, imu);

    // Fixed joint connecting power button to PCB
    SpatialTransform power_T;
    power_T.E = getRotation(0, 0, 0);
    power_T.r = Vector3d(-0.1, 0, 0.075);
    bottom_PCB.Join(power_T, power_button);
    
    // Fixed joint connecting back left motor to PCB
    SpatialTransform back_left_motor_T;
    back_left_motor_T.E = getRotation(0, M_PI_2, 0);
    back_left_motor_T.r = Vector3d(-0.110, 0.045, 0.041);
    bottom_PCB.Join(back_left_motor_T, motor);

    // Fixed joint connecting back right motor to PCB
    SpatialTransform back_right_motor_T;
    back_right_motor_T.E = getRotation(3*M_PI_2, 0, M_PI_2);
    back_right_motor_T.r = Vector3d(-0.110, -0.045, 0.041);
    bottom_PCB.Join(back_right_motor_T, motor);

    // Fixed joint connecting front left motor to PCB
    SpatialTransform front_left_motor_T;
    front_left_motor_T.E = getRotation(0, M_PI_2, 0);
    front_left_motor_T.r = Vector3d(0.091, 0.045, 0.041);
    bottom_PCB.Join(front_left_motor_T, motor);

    // Fixed joint connecting front right motor to PCB
    SpatialTransform front_right_motor_T;
    front_right_motor_T.E = getRotation(0, M_PI_2, 0);
    front_right_motor_T.r = Vector3d(0.091, -0.045, 0.041);
    bottom_PCB.Join(front_right_motor_T, motor);

    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////

    // Joint between PCB and ROOT
    Joint floating_joint(JointTypeFloatingBase);
    uint bottom_PCB_id = model->AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint, bottom_PCB, "bottom_PCB");

    // ============================================================================= //
    //                                   LEGS                                        //
    // ============================================================================= //
    

    //////////////////////////////////
    //            BODIES            //
    //////////////////////////////////


    /* ----- ALUMINUM CLAMP (m) ----- */

    double clamp_mass = 0.011;
    Vector3d clamp_com = Vector3d(0, 0, 0);
    double clamp_x = 0.02045;
    double clamp_y = 0.02625;
    double clamp_z = 0.007;
    Matrix3d clamp_inertia = cubeInertia(clamp_x, clamp_y, clamp_z, clamp_mass);
    Body clamp(clamp_mass, clamp_com, clamp_inertia);

    /* ----- ROTOR INERTIA ----- */

    Matrix3d rotor_inertia;
    double estimated_rotor_inertia = .0028;
    rotor_inertia << 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, estimated_rotor_inertia;
    Body rotor(0, Vector3d(0, 0, 0), rotor_inertia);
    cout << "Rotor Inertia: \n" << rotor_inertia << endl;

    // ----------------------------------
    // Join the rotor inertia with the clamp for ease
    SpatialTransform clamp2rotor;
    clamp2rotor.r = Vector3d(0, 0, 0);
    clamp2rotor.E = getRotation(0, 0, 0);
    clamp.Join(clamp2rotor, rotor);

    // -------------------------------------
    
    /* ------ LEFT HUB (mm) ------ */

    double left_hub_mass = 7095.798340 * pow(mm2meter,3) * thick_PLA_density;
    Vector3d left_hub_com = mm2meter * Vector3d(5.921877, 0.000081, -12.887457);
    Matrix3d left_hub_inertia;
    left_hub_inertia << 1456708.875000,  12.194215,      -446692.343750,
                        12.194215,       1472451.125000,  4.978187,
                       -446692.343750,   4.978187,        1155731.875000;
    left_hub_inertia  = scaleInteria(left_hub_inertia, thick_PLA_density);
    Body left_hub(left_hub_mass, left_hub_com, left_hub_inertia);

    // -------------------------------

    SpatialTransform lhub2clamp;
    lhub2clamp.r = Vector3d(0.015, 0, 0);
    lhub2clamp.E = getRotation(0, -M_PI_2, 0);
    left_hub.Join(lhub2clamp, clamp);

    /* ------ RIGHT HUB (mm) ------ */

    double right_hub_mass = 7095.798340 * pow(mm2meter,3) * thick_PLA_density;
    Vector3d right_hub_com = mm2meter * Vector3d(5.921877, 0.000081, 12.887457);
    Matrix3d right_hub_inertia;
    right_hub_inertia << 1456708.875000,  12.194215,        446692.343750,
                         12.194215,       1472451.125000,  -4.978187,
                         446692.343750,   -4.978187,        1155731.875000;
    right_hub_inertia  = scaleInteria(right_hub_inertia, thick_PLA_density);
    Body right_hub(right_hub_mass, right_hub_com, right_hub_inertia);

    // ------------------------------

    SpatialTransform rhub2clamp;
    rhub2clamp.r = Vector3d(0.015, 0, 0);
    rhub2clamp.E = getRotation(0, M_PI_2, 0);
    right_hub.Join(rhub2clamp, clamp);

    /* ------  UPPER LINK (mm) and MOTOR CONTROLLER (m) ------ */ 

    double upper_link_mass = 13804.297852 * pow(mm2meter,3) * thick_PLA_density;
    Vector3d upper_link_com = mm2meter * Vector3d(4.592267, 43.607891, 0.000017);
    Matrix3d upper_link_inertia;
    upper_link_inertia << 14050550.000000, 156734.750000,  0.007331,
                          156734.750000,   957646.812500, -6.240734,
                          0.007331,       -6.240734,       13227597.000000;
    upper_link_inertia = scaleInteria(upper_link_inertia, thick_PLA_density);
    Body upper_link = Body(upper_link_mass, upper_link_com, upper_link_inertia);
    
    // -------------------------------------

    SpatialTransform link2controller;
    link2controller.r = Vector3d(0.005, 0.07, 0);
    link2controller.E = getRotation(0, M_PI_2, -M_PI_2);
    upper_link.Join(link2controller, motor_controller);

    // --------------------------------------

    SpatialTransform ulink2clamp;
    ulink2clamp.r = Vector3d(0, 0, 0);
    ulink2clamp.E = getRotation(0, -M_PI_2, 0);
    upper_link.Join(ulink2clamp, clamp);

    /* ------ LEFT LOWER LINK (mm) ------ */

    double left_lower_link_mass = 13170.648438 * pow(mm2meter,3) * thick_PLA_density;
    Vector3d left_lower_link_com = mm2meter * Vector3d(-0.310779, -51.736179, -4.568526);
    Matrix3d left_lower_link_inertia;
    left_lower_link_inertia << 18151954.000000, -238616.937500, -19946.636719,
                              -238616.937500,    437603.937500, -672611.562500,
                              -19946.636719,    -672611.562500,  18307054.000000;
    left_lower_link_inertia = scaleInteria(left_lower_link_inertia, thick_PLA_density);
    Body left_lower_link(left_lower_link_mass, left_lower_link_com, left_lower_link_inertia);

    // -------------------------------------

    SpatialTransform llink2clamp;
    llink2clamp.r = Vector3d(0, 0, 0);
    llink2clamp.E = getRotation(0, 0, 0);
    left_lower_link.Join(llink2clamp, clamp);
    
    /* ------ FRONT RIGHT LOWER LINK (mm) ------ */

    double right_lower_link_mass = 13170.648438 * pow(mm2meter,3) * thick_PLA_density;
    Vector3d right_lower_link_com = mm2meter * Vector3d(-0.310779, -51.736179, 4.568526);
    Matrix3d right_lower_link_inertia;
    right_lower_link_inertia << 18151954.000000, -238616.937500, 19946.636719,
                              -238616.937500,    437603.937500, 672611.562500,
                              19946.636719,    672611.562500,  18307054.000000;
    right_lower_link_inertia = scaleInteria(right_lower_link_inertia, thick_PLA_density);
    Body right_lower_link(right_lower_link_mass, right_lower_link_com, right_lower_link_inertia);

    // -------------------------------------

    right_lower_link.Join(llink2clamp, clamp);

    /* ------ UPPER LINK SHROUD (mm) ------ */

    double leg_shroud_mass = 6413.942871 * pow(mm2meter,3) * thin_PLA_density;
    Vector3d leg_shroud_com = mm2meter * Vector3d(12.682721, 54.673603, 0.000087);
    Matrix3d leg_shroud_inertia;
    leg_shroud_inertia << 4268927.000000, 171794.656250,  0.706674,
                          171794.656250,  991662.187500, -24.252905,
                          0.706674,      -24.252905,      3827615.750000;
    leg_shroud_inertia = scaleInteria(leg_shroud_inertia, thin_PLA_density);
    Body leg_shroud(leg_shroud_mass, leg_shroud_com, leg_shroud_inertia);

    /* ------ FOOT (mm) ------ */

    double foot_mass = 0.003;
    double foot_rad  = 0.0095;
    double foot_I = 2/5 * foot_mass * SQUARE(foot_rad);
    Vector3d foot_com = mm2meter * Vector3d(0.000083, 0.000001, 14.012547);
    Matrix3d foot_inertia;
    foot_inertia << foot_I, 0,      0,
                    0,      foot_I, 0,
                    0,      0,      foot_I;
    Body foot = Body(foot_mass, foot_com, foot_inertia);


    
    //////////////////////////////////
    //            JOINTS            //
    //////////////////////////////////


    /* ---------- BACK LEFT LEG ---------- */

    // Revolute joint connecting back left hub to PCB
    Joint back_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_left_hub_T;
    back_left_hub_T.E = getRotation(-M_PI_2, 0, 0);
    back_left_hub_T.r = Vector3d(-0.147, 0.045, 0.041);
    uint back_left_hub_id = model->AddBody(bottom_PCB_id, back_left_hub_T, back_left_hub_joint, left_hub, "back_left_hub");

    // Fixed joint connecting back left shoulder motor to back left hub
    SpatialTransform back_left_shoulder_motor_T;
    back_left_shoulder_motor_T.E = getRotation(0, 0, 0);
    back_left_shoulder_motor_T.r = Vector3d(0, 0, 0.005);
    model->AddBody(back_left_hub_id, back_left_shoulder_motor_T, FIXED, motor, "back_left_shoulder_motor");

    // Revolute joint connecting back left upper link to back left hub
    Joint back_left_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform back_left_shoulder_T;
    back_left_shoulder_T.E = getRotation(0, -M_PI_2, 0);
    back_left_shoulder_T.r = Vector3d(0, 0, 0.021);
    uint back_left_upper_link_id = model->AddBody(back_left_hub_id, back_left_shoulder_T, back_left_shoulder_joint, upper_link, "back_left_upper_link");
    
    // Fixed joint connecting leg shroud to back left lower link
    SpatialTransform shroud_T;
    shroud_T.E = getRotation(M_PI, 0, 0);
    shroud_T.r = Vector3d(0.007, 0.085, 0);
    model->AddBody(back_left_upper_link_id, shroud_T, FIXED, leg_shroud, "back_left_leg_shroud");

    // Revolute joint connecting back left lower link to back left upper link
    Joint back_left_elbow_joint(JointTypeRevolute, Vector3d(0, 0, -1));
    SpatialTransform back_left_elbow_T;
    back_left_elbow_T.E = getRotation(0, -M_PI_2, M_PI);
    back_left_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint back_left_lower_link_id = model->AddBody(back_left_upper_link_id, back_left_elbow_T, back_left_elbow_joint, left_lower_link, "back_left_lower_link");

    // Fixed joint connecting motor to back left upper joint
    SpatialTransform back_left_elbow_motor_T;
    back_left_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    back_left_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(back_left_upper_link_id, back_left_elbow_motor_T, FIXED, motor, "back_left_elbow_motor");

    // Fixed joint connecting foot to back left lower link
    SpatialTransform back_left_foot_T;
    back_left_foot_T.E = getRotation(0, 0, 0);
    back_left_foot_T.r = Vector3d(0, -0.11, 0.009);
    model->AddBody(back_left_lower_link_id, back_left_foot_T, FIXED, foot, "back_left_foot");



    /* ---------- BACK RIGHT LEG ---------- */

    // Revolute joint connecting back right hub to PCB
    Joint back_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform back_right_hip_T;
    back_right_hip_T.E = getRotation(-M_PI_2, 0, 0);
    back_right_hip_T.r = Vector3d(-0.147, -0.045, 0.041);
    uint back_right_hub_id = model->AddBody(bottom_PCB_id, back_right_hip_T, back_right_hip_joint, right_hub, "back_right_hub");

    // Fixed joint connecting back right shoulder motor to back right hub
    SpatialTransform back_right_shoulder_motor_T;
    back_right_shoulder_motor_T.E = getRotation(0, 0, 0);
    back_right_shoulder_motor_T.r = Vector3d(0, 0, -0.005);
    model->AddBody(back_right_hub_id, back_right_shoulder_motor_T, FIXED, motor, "back_right_shoulder_motor");

    // Revolute joint connecting back right top link to back right hub
    Joint back_right_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform back_right_shoulder_T;
    back_right_shoulder_T.E = getRotation(0, M_PI_2, 0);
    back_right_shoulder_T.r = Vector3d(0, 0, -0.021);
    uint back_right_upper_link_id = model->AddBody(back_right_hub_id, back_right_shoulder_T, back_right_shoulder_joint, upper_link, "back_right_upper_link");

    // Fixed joint connecting back right elbow motor to back right upper link
    SpatialTransform back_right_elbow_motor_T;
    back_right_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    back_right_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(back_right_upper_link_id, back_right_elbow_motor_T, FIXED, motor, "back_right_elbow_motor");

    // Fixed joint connecting leg shroud to back right lower link
    model->AddBody(back_right_upper_link_id, shroud_T, FIXED, leg_shroud, "back_right_leg_shroud");

    // Revolute joint connecting back right lower link to back right upper link
    Joint back_right_elbow_joint(JointTypeRevolute, Vector3d(0, 0, 1));
    SpatialTransform back_right_elbow_T;
    back_right_elbow_T.E = getRotation(0, M_PI_2, M_PI);
    back_right_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint back_right_lower_link_id = model->AddBody(back_right_upper_link_id, back_right_elbow_T, back_right_elbow_joint, right_lower_link, "back_right_lower_link");

    // Fixed joint connecting back right foot to back right lower link
    SpatialTransform back_right_foot_T;
    back_right_foot_T.E = getRotation(0, 0, 0);
    back_right_foot_T.r = Vector3d(0, -0.11, -0.009);
    model->AddBody(back_right_lower_link_id, back_right_foot_T, FIXED, foot, "back_right_foot");



    /* ---------- FRONT LEFT LEG ---------- */

    // Revolute joint connecting front left hub to PCB (FL HIP)
    Joint front_left_hub_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_left_hub_T;
    front_left_hub_T.E = getRotation(-M_PI_2, 0, 0);
    front_left_hub_T.r = Vector3d(0.054, 0.045, 0.041);
    uint front_left_hub_id = model->AddBody(bottom_PCB_id, front_left_hub_T, front_left_hub_joint, left_hub, "front_left_hub");             

    // Fixed joint connecting front left shoulder motor to front left hub
    SpatialTransform front_left_shoulder_motor_T;
    front_left_shoulder_motor_T.E = getRotation(0, 0, 0);
    front_left_shoulder_motor_T.r = Vector3d(0, 0, 0.005);
    model->AddBody(front_left_hub_id, front_left_shoulder_motor_T, FIXED, motor, "front_left_shoulder_motor");

    // Revolute joint connecting front left upper link to front left hub (FL SHOULDER)
    Joint front_left_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform front_left_shoulder_T;
    front_left_shoulder_T.E = getRotation(0, -M_PI_2, 0);   
    front_left_shoulder_T.r = Vector3d(0, 0, 0.021);       
    uint front_left_upper_link_id = model->AddBody(front_left_hub_id, front_left_shoulder_T, front_left_shoulder_joint, upper_link, "front_left_upper_link");

    // Fixed joint connecting elbow motor to front left upper link
    SpatialTransform front_left_elbow_motor_T;
    front_left_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    front_left_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(front_left_upper_link_id, front_left_elbow_motor_T, FIXED, motor, "front_left_elbow_motor");

    // Fixed joint connecting leg shroud to front left lower link
    model->AddBody(front_left_upper_link_id, shroud_T, FIXED, leg_shroud, "front_left_leg_shroud");

    // Revolute joint connecting front left upper link and front left lower link (FL ELBOW)
    Joint front_left_elbow_joint = Joint(JointTypeRevolute, Vector3d(0, 0, -1));
    SpatialTransform front_left_elbow_T;
    front_left_elbow_T.E = getRotation(0, -M_PI_2, M_PI);
    front_left_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint front_left_lower_link_id = model->AddBody(front_left_upper_link_id, front_left_elbow_T, front_left_elbow_joint, left_lower_link, "front_left_lower_link");
    
    // Fixed joint connecting foot to lower link
    SpatialTransform left_foot_T;
    left_foot_T.E = getRotation(0, 0, 0);
    left_foot_T.r = Vector3d(0, -0.11, 0.009);
    model->AddBody(front_left_lower_link_id, left_foot_T, FIXED, foot, "front_left_foot");



    /* ---------- FRONT RIGHT LEG ---------- */

    // Revolute joint connecting front right hub to PCB (FR HIP)
    Joint front_right_hip_joint(JointTypeRevolute, Vector3d(-1, 0, 0));
    SpatialTransform front_right_hub_T;
    front_right_hub_T.E = getRotation(-M_PI_2, 0, 0);
    front_right_hub_T.r = Vector3d(0.054, -0.045, 0.041);
    uint front_right_hub_id = model->AddBody(bottom_PCB_id, front_right_hub_T, front_right_hip_joint, right_hub, "front_right_hub");

    // Fixed joint connecting motor to front right hub
    SpatialTransform front_right_shoulder_motor_T;
    front_right_shoulder_motor_T.E = getRotation(0, 0, 0);
    front_right_shoulder_motor_T.r = Vector3d(0, 0, -0.005);
    model->AddBody(front_right_hub_id, front_right_shoulder_motor_T, FIXED, motor, "front_right_shoulder_motor");

    // Revolute joint connecting front right upper link to front right hub (FR SHOULDER)
    Joint front_right_shoulder_joint(JointTypeRevolute, Vector3d(1, 0, 0));
    SpatialTransform front_right_upper_link_T;
    front_right_upper_link_T.E = getRotation(0, M_PI_2, 0);
    front_right_upper_link_T.r = Vector3d(0, 0, -0.021);
    uint front_right_upper_link_id = model->AddBody(front_right_hub_id, front_right_upper_link_T, front_right_shoulder_joint, upper_link, "front_right_upper_link");

    // Fixed joint connecting elbow motor with front rigth upper link
    SpatialTransform front_right_elbow_motor_T;
    front_right_elbow_motor_T.E = getRotation(0, M_PI_2, 0);
    front_right_elbow_motor_T.r = Vector3d(0.023, 0.082, 0);
    model->AddBody(front_right_upper_link_id, front_right_elbow_motor_T, FIXED, motor, "front_right_elbow_motor");

    // Fixed joint connecting leg shroud to front right lower link
    model->AddBody(front_right_upper_link_id, shroud_T, FIXED, leg_shroud, "front_right_leg_shroud");

    // Revolute joint connection front right lower link to front right upper link (FR ELBOW)
    Joint front_right_elbow_joint(JointTypeRevolute, Vector3d(0, 0, 1));
    SpatialTransform front_right_elbow_T;
    front_right_elbow_T.E = getRotation(0, M_PI_2, M_PI);
    front_right_elbow_T.r = Vector3d(-0.006, 0.08, 0);
    uint front_right_lower_link_id = model->AddBody(front_right_upper_link_id, front_right_elbow_T, front_right_elbow_joint, right_lower_link, "front_right_lower_link");

    // Fixed joint connecting front right foot to front right lower link
    SpatialTransform front_right_foot_T;
    front_right_foot_T.E = getRotation(0, 0, 0);
    front_right_foot_T.r = Vector3d(0, -0.11, -0.009);
    uint front_right_foot_id = model->AddBody(front_right_lower_link_id, front_right_foot_T, FIXED, foot, "front_right_foot");

    printCOM(*model);

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



// Old tests saved for posterity:
// std::shared_ptr<Model> createPupperModel(){

//     auto model = std::make_shared<Model>();

//     // Z direction is up
//     model->gravity = Vector3d(0, 0, -9.81);

// //* -----------------------------TESTING---------------------------------*//
// // Answering questions:
// // 1. For fixed bodies, do the intertias have to be in the same frame or no?
// //     a. Join two bodies with the same frame -> print mass matrix
// //     b. Join two bodies with different frames -> print mass matrix
// //     c. mass matrix different? Then inertia's DONT have to be in the same frame. (GOOD)
// //        Answer: No, inertia's don't have to be in the same frame (the inertia of the child is described in the child's frame 
// //                and can differ from the parent frame) 
// // 2. COM described in parent or child??
// //     a. Add body at 0,0,0 with COM at 0,0,0 -> print mass matrix
// //     b. Add body at 1000,1000,1000 with COM at 0,0,0 -> print mass matrix
// //     c. Mass matrix different? Then COM is described in child frame. (GOOD)
// //        Answer: Yes, COM of added body is described in child frame.
//  /* ------ BOTTOM PCB (inches) ------ */
//     // TEST 1 /////////////////////////

//     // Create the bottom PCB
//     double pcb_mass = 0.0246155987687;
//     Vector3d pcb_com = inch2meter * Vector3d(-0.134500, -0.000013, -0.000000);
//     Matrix3d pcb_inertia;
//     pcb_inertia << 10,       0,          0, 
//                    0,        10,         0,
//                    0,        0,         10;
    
//     // Joint between PCB and ROOT
//     Joint floating_joint(JointTypeFloatingBase);

//     // Add PCB to body
//     Body bottom_PCB = Body(pcb_mass, pcb_com, pcb_inertia);
//     uint bottom_PCB_id = model->AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint, bottom_PCB, "bottom_PCB");

//     /* ------ TEST BODY 2 (mm) ------ */

//     double front_bulkhead_mass = 28243.179688 * pow(mm2meter,3) * density;
//     Vector3d front_bulkhead_com = mm2meter * Vector3d(0, 0, 0);
//     Matrix3d front_bulkhead_inertia;
//     front_bulkhead_inertia << 10,       0,          0, 
//                               0,        0,          0,
//                                0,        0,         0;

//     // Fixed joint connecting TEST BODY 2 to PCB
//     Joint fixed_joint_floating_bulkhead(JointTypeFixed);
//     SpatialTransform front_bulkhead_T;
//     front_bulkhead_T.E << 1, 0, 0, // 1.a  2.a,b
//                           0, 1, 0,
//                           0, 0, 1; 
//     // front_bulkhead_T.E << 0,-1, 0, // 1.b
//     //                       1, 0, 0,
//     //                       0, 0, 1; 
//     cout << "TestBody2 T.E = \n" << front_bulkhead_T.E <<endl;
//     //front_bulkhead_T.r = Vector3d(0, 0, 0); // 1.a,b ,  2.a
//     front_bulkhead_T.r = Vector3d(1000, 1000, 1000); // 2.b

//     // Add TEST BODY 2 to body
//     Body front_bulkhead = Body(front_bulkhead_mass, front_bulkhead_com, front_bulkhead_inertia);
//     uint front_bulkhead_id = model->AddBody(bottom_PCB_id, front_bulkhead_T, fixed_joint_floating_bulkhead, front_bulkhead, "front_bulkhead");

//     ///////////////////////////////////////////////////////////////
//     cout << "\n---------- DYNAMICS TESTING ------------\n" << endl;
//     VectorNd q(model->q_size); q.setZero();
//     MatrixNd M(model->qdot_size, model->qdot_size); M.setZero();
//     RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, q, M);
//     cout << "\nMass Matrix: \n" << M << endl;


// // Creating a simple model root -> floating joint -> body with another body fixed offset from origin
    // */
    // Model model_test;
    // model_test.gravity = Vector3d(0, 0, -9.81);
    // double bottom_pcb_mass_test = 0.1; // kg

    // // Create the bottom PCB
    // Vector3d pcb_com_test = Vector3d(0.0,0.0,0.0);
    // Matrix3d pcb_inertia_test;
    // pcb_inertia_test << 0.001, 0,         0, 
    //                     0,        0.001, 0,
    //                     0,        0,         0.001;
    // Body bottom_PCB_test = Body(bottom_pcb_mass_test, pcb_com_test, pcb_inertia_test);

    // // Joint between PCB and ROOT
    // Joint floating_joint_test(JointTypeFloatingBase);
    // uint bottom_PCB_id_test = model_test.AddBody(0, Xtrans(Vector3d(0, 0, 0)), floating_joint_test, bottom_PCB_test, "bottom_PCB_test");

    // // Add Battery to bottom PCB
    // double batt_mass_test = 0.2; // Kg
    // Vector3d batt_com_test(0, 0, 0);
    // Matrix3d batt_inertia_test;
    // batt_inertia_test << 0.001, 0,         0, 
    //                     0,        0.001, 0,
    //                     0,        0,         0.001;
    // Body battery_test(batt_mass_test, batt_com_test, batt_inertia_test);

    // // Fixed joint connecting battery to bottom PCB
    // SpatialTransform battery_T_test;
    // battery_T_test.E = getRotation(0, 0, 0);
    // battery_T_test.r = Vector3d(-0.05, 0.0, 0); // m
    // model_test.AddBody(bottom_PCB_id_test, battery_T_test, FIXED, battery_test, "battery_test");

    // VectorNd q_test(model_test.q_size);
    // VectorNd q_dot_test(model_test.qdot_size);
    // q_test.setZero();
    // q_test(model_test.q_size-1) = 1.0;

    // // Solve for NonlinearEffects
    // VectorNd b_g_test(model_test.dof_count);
    // cout << "dof_count: " << model_test.dof_count << endl;
    // b_g_test.setZero(); 
    // cout << "q angles: " << q_test.transpose() << endl;
    // cout << "q_dot: " << q_dot_test.transpose() << endl;
    // NonlinearEffects(model_test, q_test, q_dot_test, b_g_test);
    // cout << "b_g_ NonlinearEffects Test:" << b_g_test.transpose() << endl;
    // cout << "TEST OF SIMPLE MODEL DONE \n\n\n" << endl;
    // ///////////////////////////END//////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////RESULTS////////////////////////////////////////////////////
// //
// ////////////////////////////////////////////////////////////////////////////////////////////////