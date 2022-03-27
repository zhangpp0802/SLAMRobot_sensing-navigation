#ifndef TURTLELIB_INCLUDE_GUARD_HPP
#define TURTLELIB_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
    	return (fabs(d1-d2) < epsilon); 
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \return radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
    	return deg*PI/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \return the angle in degrees
    constexpr double rad2deg(double rad)
    {
    	return rad*180/PI;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");



    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
    	/// x component of vector
        double x = 0.0;
        /// y component of vector  
        double y = 0.0;  
        
        /// \brief Create a 0 vector
        Vector2D();
        
        /// \brief Create a vector with prescribed x and y components
        /// \param x_arg - x component of vector
        /// \param y_arg - y component of vector
        Vector2D(double x_arg, double y_arg);
        
        /// \brief make the Vector2D instance a unit vector if not zero and return it
        /// \return the normalized Vector2D instance 
        Vector2D & normalize();
        
        /// \brief add this vector to another and store the result 
        /// in this object
        /// \param rhs - the vector to add
        /// \return a reference to the newly added vector
        Vector2D & operator+=(const Vector2D & rhs);
        
        /// \brief subtract another vector from this and store the result 
        /// in this object
        /// \param rhs - the vector to add
        /// \return a reference to the newly added vector
        Vector2D & operator-=(const Vector2D & rhs);
        
        /// \brief multiply this vector by a scalar
        /// \param s - the scalar to multiply by
        /// \return a reference to the scaled vector
        Vector2D & operator*=(const double s);
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    std::istream & operator>>(std::istream & is, Vector2D & v);
    
    /// \brief perform vector addition on Vector2D elements 
    /// \param lhs - the first vector to add
    /// \param rhs - the second vector to add
    /// \return the result of the vector addition
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);
    
    /// \brief perform vector subtraction on Vector2D elements 
    /// \param lhs - the first vector
    /// \param rhs - the vector to subtract
    /// \return the result of the vector subtraction
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);  
    
    /// \brief multiply a vector by a scalar
    /// \param v - the vector to multiply
    /// \param s - the scalar
    /// \return the scaled vector
    Vector2D operator*(Vector2D v, const double s);
    
    /// \brief multiply a vector by a scalar
    /// \param v - the vector to multiply
    /// \param s - the scalar
    /// \return the scaled vector
    Vector2D operator*(const double s, Vector2D v);
    
    /// \brief return the magnitude of a vector
    /// \param v - a Vector2D object
    /// \return the magnitude of the vector
    double magnitude(const Vector2D & v); 
    
    /// \brief return the angle of a vector
    /// \param v - a Vector2D object
    /// \return the angle of the vector
    double angle(const Vector2D & v); 
    
    /// Forward declaration of Twist2D
    class Twist2D;

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
    	/// 2D translation vector
    	Vector2D m_trans;
    	/// rotation angle about the z axis in radians   
    	double m_radians;  
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);
        
        /// \brief get the translation component in the x direction
        /// \return the translation component in the x direction
        double getX() const;
        
        /// \brief get the translation component in the y direction
        /// \return the translation component in the y direction
        double getY() const;
        
        /// \brief get the sin of the rotation angle
        /// \return the sin of the rotation angle
        double getStheta() const;
        
        /// \brief get the cos of the rotation angle
        /// \return the cos of the rotation angle
        double getCtheta() const;
        
        /// \brief get the the rotation angle
        /// \return the rotation angle
        double getTheta() const;

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;
        
        /// \brief Convert twist to a different reference frame using the adjoint of the object's transformation
        /// \param tw - the twist to convert
        /// \return a twist in the new reference frame
        Twist2D change_twist_frame(const Twist2D & tw) const; 
        
        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
    
    /// \brief a twist in 2 dimensions
    class Twist2D
    {
    private:
    	/// 2D translational velocity vector
    	Vector2D m_trans_v;
    	/// rotational velocity about the z axis (in radians/time)  		
    	double m_rot_v;  	
    	/// allow Transform2D to access the private members of Twist2D		
    	friend class Transform2D;  	
	public:
		/// \brief Create a 0 velocity twist
        Twist2D();

        /// \brief create a twist that is a pure translational velocity vector
        /// \param trans_v - the translational velocity vector
        explicit Twist2D(const Vector2D & trans_v);

        /// \brief create a twist that is a pure rotational velocity
        /// \param rot_v - the rotational velocity (in radians/time)
        explicit Twist2D(double rot_v);
        
        /// \brief get the x linear component of the 2D twist
        /// \return the x linear component of the 2D twist
        double getVx() const;
        
        /// \brief get the y linear component of the 2D twist
        /// \return the y linear component of the 2D twist
		double getVy() const;
		
		/// \brief get the rotational component of the 2D twist
        /// \return the rotational component of the 2D twist
		double getW() const;

        /// \brief Create a twist with a translational and rotational
        /// component
        /// \param trans_v - the translational velocity 
        /// \param rot_v - the rotational velocity (in radians/time)
        Twist2D(const Vector2D & trans_v, double rot_v);
        
		/// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);
    };

    /// \brief should print a human readable version of the twist
    /// An example output:
    /// w: 2 x_dot: 3 y_dot: 5
    /// \param os - an output stream
    /// \param tw - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);
    
    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (w, x_dot, y_dot) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Twist2D & tw);
    
    /// \brief converts an angle to the range [-pi, pi]
    /// \param rad - the angle to convert in radians
    /// \return an angle in radians between [-pi, pi]
    double normalize_angle(double rad);
    
    /// \brief computes the difference between two angles in the range [-pi, pi] assuming small angle displacement
    /// \param th_new - the new angular position
    /// \param th_old - the previous angular position (to be subtracted)
    /// \return the smallest angular difference
    double normalize_angular_difference(double th_new, double th_old);
    
    /// \brief computes the transformation corresponding to a rigid body following
    /// a twist for unit time
    /// \param tw - the twist followed by the transformation
    /// \return a transformation due to the twist
    Transform2D integrateTwist(Twist2D & tw);
}

#endif

