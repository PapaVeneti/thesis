#include <eigen3/Eigen/Dense>

#define qMH_offset -M_PI_2
#define q11_offset  2.3095
#define q21_offset  1.3265
#define q12_offset  0.83482
#define q22_offset  -1.3233

#define z_MH_j11j21 -0.0597

//Limits
// qMH:
#define qMH_lb -M_PI
#define qMH_ub  M_PI
// q11:
#define q11_lb -M_PI
#define q11_ub  1.65
// q21:
#define q21_lb -2.5
#define q21_ub  1.58
// q12:
#define q12_lb -1.65
#define q12_ub  M_PI
// q22:
#define q22_lb -1.51
#define q22_ub  2.5

#define SQUARE(x) ((x)*(x))
#define ANLGE_DIST(x1,x2,x3,x4)  SQUARE(x1)+SQUARE(x2)+SQUARE(x3)+SQUARE(x4)


// enum config_parameter {FR_RR}

enum joint_id {jMH,j11,j21,j12,j22};

class position_controller {
  public:		
    position_controller(int configuration):config_param(configuration)  {
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
    Eigen::Vector3d DK(const Eigen::Vector3d &JointPositionVector);

    ///Function to return the full  leg state.
    /// @note It must be called after the most relevatn DK call
    Eigen::Matrix<double,5,1> calculate_joint_angles(const Eigen::Vector3d &JointPositionVector);

    /// FW kinematics based on the current measured state `qm`
    /// It updated the current estimated end effector position `pEE` in the {MH} frame.
    void DK();
    void calculate_joint_angles();
    void state_estimation();


    //getters
    Eigen::Vector3d get_EE_position(void);
    Eigen::Matrix<double,5,1> get_joint_angles(void);
    
    
    //helpers:
    Eigen::Matrix3d rotate_x(const double & th);


    // IK

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
    Eigen::Vector3d IK(const Eigen::Vector3d & pD_MH  );

    /// @brief Returns the solutions of the inverse kinematics for an RR manipulator
    ///
    /// @param[in] 
    /// @return An `std::array` with 2 arrays for the 2 different solutions. Each array contains 2 angles `q1i` and `q2i` where i=1,2 for each chain.
    /// @note The leg doesn't know its position in the world frame. The user is responsible for further transformations. 
    /// 
    std::array<std::array<double,2>,2> RR_IK(const double &x,const double &y, const bool & chain_1);
    inline double RR_IK_system(const double x,const double y,const double r1,const double r2,const double & theta2);

    void feasibility_check( 
      const std::array<std::array<double,2>,2> & SOLS_1,
      const std::array<std::array<double,2>,2> & SOLS_2 );

    std::array<int,2> RR_best_sol( 
    const std::array<std::array<double,2>,2> & SOLS_1,
    const std::array<std::array<double,2>,2> & SOLS_2 );
    inline double angle_distance(const std::array<double,2>& sol_chain1,const std::array<double,2>& sol_chain2,const Eigen::Matrix<double,5,1> qpos );
    
    inline bool joint_angle_in_limits(const double & q, const std::array<double,2> & qlim);
    inline bool solution_in_limits(const std::array<double,2> & sol, const std::array<double,2> & qa_lim, const std::array<double,2> & qb_lim);

// private:
public:
//Geometrical Quantites:
double j11_Dx = 0.046; //Horizontal distance of joint11 from MH frame
double l11    = 0.18;
double l21    = 0.29977;

double j12_Dx = 0.136; //Horizontal distance of joint12 from MH frame
double l12    = 0.18;
double l22    = 0.29929;

double j_Dy = -1.1338e-05; //Vertical Distance of joint11 and joint12 from MH frame

//Kinematic Quantities
const int config_param = 2; // [config 1: {FR,RL}, config2: {FL,RR}] 

Eigen::Vector2d       p1c,p2c;                   //`p1c`,`p2c` are circle centers in the rotated {MH} frame. Used in `DK`, and saved for state estimation
Eigen::Vector2d       p_EE_22d;  //End Effector position vector in the rotated {MH} frame -> in the projected plane -> 2d.
const Eigen::Vector2d pj11{0.046, -1.1338e-05} ; // `pj11` is the center of joint11
const Eigen::Vector2d pj12{0.136, -1.1338e-05} ; // `pj12` is the center of joint12 

//State (Cartesian and Angle joints)
Eigen::Vector3d pEE; //End Effector position vector in the default {MH} frame.
Eigen::Vector3d qm;           //Measured Joint Angles
Eigen::Matrix<double,5,1> q;  //Joint Angles
Eigen::Matrix<double,5,1> qd; //Desired Joint Angles

//limits
const std::array<double,2> qMH_lim { qMH_lb,qMH_ub};
const std::array<double,2> q11_lim { q11_lb,q11_ub};
const std::array<double,2> q21_lim { q21_lb,q21_ub};
const std::array<double,2> q12_lim { q12_lb,q12_ub};
const std::array<double,2> q22_lim { q22_lb,q22_ub};


// Boolean array for the feasibility checks. (s = solution, i=chain_index)
// { 
//   [s1[0] + s2[0],  s1[0] + s2[1] ], 
//   [s1[1] + s2[0],  s1[1] + s2[1] ] 
// }
std::array<std::array<bool,2>,2> feasibility_matrix;
const double qMH_default_angle =  -M_PI_2;


const std::array<double,5> offsets_1 ={qMH_offset,q11_offset ,q21_offset,q12_offset ,q22_offset}; //SOS MUST CHANGE FOR CONFIG 2
const std::array<double,5> offsets_2 ={qMH_offset,-q11_offset,q21_offset,-q12_offset,q22_offset}; //SOS MUST CHANGE FOR CONFIG 2
const Eigen::Vector3d p_MH_j11{0.046,-1.1338e-05,z_MH_j11j21};
// const Eigen::Vector3d p_MH_j11{0.046, 1.1338e-05,z_MH_j11j21};
};

