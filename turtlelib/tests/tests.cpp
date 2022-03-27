// imported from githubfake_turtle
#include <catch_ros/catch.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

TEST_CASE("Default constructor creates identity transform","[identity_constructor]"){ 
    using namespace turtlelib;

    Transform2D identity = Transform2D();
        
    REQUIRE(almost_equal(identity.getCtheta(),1));
    REQUIRE(almost_equal(identity.getStheta(),0));
    REQUIRE(almost_equal(identity.getX(),0));
    REQUIRE(almost_equal(identity.getY(),0));
}

TEST_CASE("Constructor using only radians component","[rad_constructor]"){ 
    using namespace turtlelib;

    Transform2D trans = Transform2D(PI);
        
    REQUIRE(almost_equal(trans.getCtheta(),cos(PI)));
    REQUIRE(almost_equal(trans.getStheta(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),0));
    REQUIRE(almost_equal(trans.getY(),0));
}

TEST_CASE("Constructor using only vector component","[vector_constructor]"){
    using namespace turtlelib;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v);
        
    REQUIRE(almost_equal(trans.getCtheta(),1));
    REQUIRE(almost_equal(trans.getStheta(),0));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

TEST_CASE("Constructor using both components","[full_constructor]"){
    using namespace turtlelib;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v,PI);
        
    REQUIRE(almost_equal(trans.getCtheta(),cos(PI)));
    REQUIRE(almost_equal(trans.getStheta(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

TEST_CASE("Inverse of Transform Matrix", "[transform]"){
    using namespace turtlelib;

    Vector2D v;
    v.x = 1;
    v.y = 2;

    Transform2D transMat = Transform2D(v, PI);
    Transform2D invTransMat = transMat.inv();

    REQUIRE(almost_equal(invTransMat.getCtheta(), cos(PI)));
    REQUIRE(almost_equal(invTransMat.getStheta(), -sin(PI)));
    REQUIRE(almost_equal(invTransMat.getX(), 1));
    REQUIRE(almost_equal(invTransMat.getY(), 2));
}

TEST_CASE("Input a Transform from istream","[input]"){
    using namespace turtlelib;

    Transform2D trans;

    std::unique_ptr<std::istream> is;
    is = std::make_unique<std::istringstream>(std::istringstream{"-30 2 3"});
    *is >> trans;

    REQUIRE(almost_equal(trans.getCtheta(),cos(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getStheta(),sin(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

TEST_CASE("Input a Transform from istream, cout format","[input]"){ 
    using namespace turtlelib;

    Transform2D trans;

    std::unique_ptr<std::istream> is;
    is = std::make_unique<std::istringstream>(std::istringstream{"dtheta (degrees): -30 dx: 2 dy: 3"});
    *is >> trans;

    REQUIRE(almost_equal(trans.getCtheta(),cos(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getStheta(),sin(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

TEST_CASE("Outpt a Transform to ostream","[output]"){
    using namespace turtlelib;

    Transform2D trans = Transform2D();
    double test;
    std::stringstream ss;
    ss << trans;

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

}

TEST_CASE("Transformation applied to Vector2D", "[transform]") 
{
    using namespace turtlelib; 
    Vector2D v; 
    v.x = -1;
    v.y = 2;

    Transform2D trans21 = Transform2D(v,PI); 

    Vector2D v1; 
    v1.x = -3; 
    v1.y = 1; 

    Vector2D v2; 
    v2 = trans21.operator()(v1);

    REQUIRE(almost_equal(v2.x, 2)); 
    REQUIRE(almost_equal(v2.y, 1));
}

TEST_CASE("Transform Composition Operator (*=)", "[transform]"){ 
    using namespace turtlelib;

    Vector2D v;
    v.x = 3;
    v.y = 9;

    Transform2D transMat = Transform2D(v, PI/9);
    Transform2D invTransMat = transMat.inv();

    transMat*=invTransMat;

    REQUIRE(almost_equal(transMat.getCtheta(), 1));
    REQUIRE(almost_equal(transMat.getStheta(), 0));
    REQUIRE(almost_equal(transMat.getX(), 0));
    REQUIRE(almost_equal(transMat.getY(), 0));
}

TEST_CASE("Change twist reference frame", "[transform]"){
	using namespace turtlelib;
	
	Vector2D p;
	p.x = 1;
	p.y = 2;
	
	Transform2D transMat = Transform2D(p, PI/2);
	Transform2D invTransMat = transMat.inv();
	
	Vector2D v;
	v.x = 3;
	v.y = 5;
	
	Twist2D tw(v, 2);
	Twist2D tw_out = invTransMat.change_twist_frame(tw);
	REQUIRE(almost_equal(tw_out.getW(), 2));
	REQUIRE(almost_equal(tw_out.getVx(), 7));
	REQUIRE(almost_equal(tw_out.getVy(), 1));
}

TEST_CASE("Access methods for Transform2D", "[access]"){ 
	using namespace turtlelib;
	
	Vector2D v;
	v.x = -1;
	v.y = 2.4;
	
	Transform2D transMat = Transform2D(v, PI);
	REQUIRE(almost_equal(transMat.getX(), -1));
	REQUIRE(almost_equal(transMat.getY(), 2.4));
	REQUIRE(almost_equal(transMat.getCtheta(), -1));
	REQUIRE(almost_equal(transMat.getStheta(), 0));
}

TEST_CASE("Vector2D addition", "[operator]") 
{
	using namespace turtlelib;
	
	Vector2D v1(1.0, 3.0);
	Vector2D v2(-2.0, 4.5);
	Vector2D v3 = v1 + v2;
	REQUIRE(almost_equal(v3.x, -1.0));
	REQUIRE(almost_equal(v3.y, 7.5));
}

TEST_CASE("Vector2D subtraction", "[operator]") 
{ 
	using namespace turtlelib;
	
	Vector2D v1(1.0, 3.0);
	Vector2D v2(-2.0, 4.5);
	Vector2D v3 = v1 - v2;
	REQUIRE(almost_equal(v3.x, 3.0));
	REQUIRE(almost_equal(v3.y, -1.5));
}

TEST_CASE("Vector2D multiplication by a scalar", "[operator]") 
{
	using namespace turtlelib;
	
	Vector2D v(2.0, -1.0);
	double s = 3.0;
	v *= s;
	REQUIRE(almost_equal(v.x, 6.0));
	REQUIRE(almost_equal(v.y, -3.0));
}

TEST_CASE("Vector2D constructor", "[constructor]") 
{
	using namespace turtlelib;
	
	Vector2D v(3.1, -1.2);
	REQUIRE(almost_equal(v.x, 3.1));
	REQUIRE(almost_equal(v.y, -1.2));
}

TEST_CASE("Vector2D default constructor", "[constructor]")
{
	using namespace turtlelib;
	
	Vector2D v;
	REQUIRE(almost_equal(v.x, 0));
	REQUIRE(almost_equal(v.y, 0));
}

TEST_CASE("Vector2D magnitude", "[vector_properties]") 
{
	using namespace turtlelib;
	
	Vector2D v(3.0, 4.0);
	REQUIRE(almost_equal(magnitude(v), 5.0));
}

TEST_CASE("Vector2D angle", "[vector_properties]")
{
	using namespace turtlelib;
	
	Vector2D v1(4.0, 4.0);
	Vector2D v2(-2.0, 0.0);
	Vector2D v3(0.0, -5.0);
	REQUIRE(almost_equal(angle(v1), PI/4));
	REQUIRE(almost_equal(angle(v2), PI));
	REQUIRE(almost_equal(angle(v3), -PI/2));
}

TEST_CASE("Normalize angle", "[angle_operations]") 
{
	using namespace turtlelib;
	
	double th1 = 2*PI + PI/6;
	double th2 = -PI/2;
	double th3 = -3*PI/2;
	double th4 = -2*PI - PI/6;
	REQUIRE(almost_equal(normalize_angle(th1), PI/6));
	REQUIRE(almost_equal(normalize_angle(th2), -PI/2));
	REQUIRE(almost_equal(normalize_angle(th3), PI/2));
	REQUIRE(almost_equal(normalize_angle(th4), -PI/6));
}

TEST_CASE("Twist integration", "[transform]") 
{
	using namespace turtlelib;
	
	Vector2D v(3.0, 2.0);
	Twist2D tw_trans(v);
	Transform2D tf_trans = integrateTwist(tw_trans);
	REQUIRE(almost_equal(tf_trans.getX(), 3.0));
	REQUIRE(almost_equal(tf_trans.getY(), 2.0));
	REQUIRE(almost_equal(tf_trans.getTheta(), 0.0));
		
	double w = 0.5;
	Twist2D tw_rot(w);
	Transform2D tf_rot = integrateTwist(tw_rot);
	REQUIRE(almost_equal(tf_rot.getX(), 0.0));
	REQUIRE(almost_equal(tf_rot.getY(), 0.0));
	REQUIRE(almost_equal(tf_rot.getTheta(), 0.5));
	
	w = PI/2;
	Twist2D tw_full(v, w);
	Transform2D tf_full = integrateTwist(tw_full);
	REQUIRE(almost_equal(tf_full.getX(), 2.0/PI));
	REQUIRE(almost_equal(tf_full.getY(), 10.0/PI));
	REQUIRE(almost_equal(tf_full.getTheta(), PI/2));
}	

TEST_CASE("Odometry pose update for pure translation", "[odometry]") 
{
	using namespace turtlelib;
	double radius = 1.0;
	DiffDrive dd(2.0, radius);
	dd.updatePoseWithSlip(PI/4, PI/4);
	REQUIRE(almost_equal(dd.getX(), 2*radius*PI/8));
	REQUIRE(almost_equal(dd.getY(), 0.0));
	REQUIRE(almost_equal(dd.getTheta(), 0.0));
}

TEST_CASE("Odometry pose update for pure rotation", "[odometry]")
{
	using namespace turtlelib;
	double base = 2.0;
	double radius = 1.0;
	DiffDrive dd(base, radius);
	dd.updatePoseWithSlip(-PI/4, PI/4);
	REQUIRE(almost_equal(dd.getX(), 0.0));
	REQUIRE(almost_equal(dd.getY(), 0.0));
	REQUIRE(almost_equal(dd.getTheta(), PI/4));
}

TEST_CASE("Twist to wheel angular velocity conversion for pure linear velocity", "[odometry]") 
{
	using namespace turtlelib;
	double lin_speed = 3.0;
	Vector2D v(lin_speed, 0.0);
	Twist2D tw(v, 0.0);
	double base = 2.0;
	double radius = 1.0;
	DiffDrive dd(base, radius);
	WheelVel wv = dd.twist2Wheel(tw);
	REQUIRE(almost_equal(wv.left, lin_speed/radius));
	REQUIRE(almost_equal(wv.right, lin_speed/radius));
} 

TEST_CASE("Twist to wheel angular velocity conversion for pure rotational velocity", "[odometry]") 
{
	using namespace turtlelib;
	double rot_speed = 3.0;
	Vector2D v(0.0, 0.0);
	Twist2D tw(v, rot_speed);
	double base = 2.0;
	double radius = 1.0;
	DiffDrive dd(base, radius);
	WheelVel wv = dd.twist2Wheel(tw);
	REQUIRE(almost_equal(wv.left, -0.5*rot_speed*base/radius));
	REQUIRE(almost_equal(wv.right, 0.5*rot_speed*base/radius));
} 
	 
