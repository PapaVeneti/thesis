#include <eigen3/Eigen/Dense>

#define q11_offset  2.3095
#define q21_offset  1.3265
#define q12_offset  0.83482
#define q22_offset  -1.3233

#define z_MH_j11j21 -0.0597



class position_controller {
  public:		
    position_controller() {
        ;
    }
    
    /// @brief Return the position of the end effector in the `MH` frame. 
    ///
    /// [joint1,joint2,motor3]
    /// @param[in] JointPositionVector Vector containing the joint angles of the leg
    /// @return Position in MotorHousing frame (=motor3 frame in urdf for zero angle) to debug!
    /// @note The leg doesn't know its position in the world frame. The user is responsible for further transformations. 
    /// Based on: "The Pantograph Mk-II: A Haptic Instrument". 
    ///
    void DK(const Eigen::Vector3d &JointPositionVector);

    Eigen::Vector3d get_EE_position(void);

    Eigen::Matrix3d rotate_x(const double & th);

    void calculate_joint_angles(const Eigen::Vector3d &JointPositionVector);

    Eigen::Matrix<double,5,1> get_joint_angles(void);







      /// @brief Return the input in`motor3` frame. 
    ///
    /// Returns [joint1,joint2,motor3]
    /// @brief Return the joint angles for a desired position of the end effector in the `motor3` frame. 
    ///
    /// [joint1,joint2,motor3]
    /// @param[in] p_M3 an `Eigen::Vector3d` containing the desired position of the leg in `motor3` frame
    /// @return Desired joint angles (MUST ADD SECOND CONFIGURATION)
    /// @note The leg doesn't know its position in the world frame. The user is responsible for further transformations. 
    /// 
    /// TO DO: 
    /// 1. add second configuration.
    /// 2. Add checks for valid desired positions. 
    ///
    Eigen::Vector3d IK(const Eigen::Vector3d & p_M3 );

private:
//Geometrical Quantites:
double j11_Dx = 0.046; //Horizontal distance of joint11 from MH frame
double l11    = 0.18;
double l21    = 0.29977;

double j12_Dx = 0.136; //Horizontal distance of joint12 from MH frame
double l12    = 0.18;
double l22    = 0.29929;

double j_Dy = -1.1338e-05; //Vertical Distance of joint11 and joint12 from MH frame

//Kinematic Quantities
double side_sign =-1; //-1 for right side, +1 for left -> in constructor 
Eigen::Vector2d       p1c,p2c;                   //`p1c`,`p2c` are circle centers in the rotated {MH} frame. Used in `DK`, and saved for state estimation
const Eigen::Vector2d pj11{0.046, -1.1338e-05} ; // `pj11` is the center of joint11
const Eigen::Vector2d pj12{0.136, -1.1338e-05} ; // `pj12` is the center of joint12 
Eigen::Vector2d       p_EE_22d;  //End Effector position vector in the rotated {MH} frame -> in the projected plane -> 2d.

//State (Cartesian and Angle joints)
Eigen::Vector3d pEE; //End Effector position vector in the default {MH} frame.
Eigen::Matrix<double,5,1> q;  //Joint Angles
Eigen::Matrix<double,5,1> qd; //Desired Joint Angles

};