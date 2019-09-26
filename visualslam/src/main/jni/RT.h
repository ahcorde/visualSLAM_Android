#ifndef RT_H
#define RT_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <cassert>
#include "eigen3/Eigen/Dense"


/// Class to represent a three-dimensional rotation matrix. Three-dimensional rotation
/// matrices are members of the Special Orthogonal Lie group mySO3. This group can be parameterised
/// three numbers (a vector in the space of the Lie Algebra). In this class, the three parameters are the
/// finite rotation vector, i.e. a three-dimensional vector whose direction is the axis of rotation
/// and whose length is the angle of rotation in radians. Exponentiating this vector gives the matrix,
/// and the logarithm of the matrix gives this vector.
/// @ingroup gTransforms
class mySO3 {
public:

    mySO3(): my_matrix(Eigen::Matrix3d::Identity()) {}

	/// Assignment operator from a general matrix. This also calls coerce()
	/// to make sure that the matrix is a valid rotation matrix.
    mySO3& operator=(const Eigen::Matrix3d& rhs) {
        my_matrix = rhs;
        coerce();
        return *this;
    }

	/// Modifies the matrix to make sure it is a valid rotation matrix.
	void coerce() {
        Eigen::Vector3d v0,v1,v2;

        //First row
            my_matrix.block(0,0,1,3).normalize();
        //Second row
        v0 = my_matrix.block(0,0,1,3).transpose();
        v1 = my_matrix.block(1,0,1,3).transpose();
            my_matrix.block(1,0,1,3) -= my_matrix.block(0,0,1,3) * (v0.dot(v1));
            my_matrix.block(1,0,1,3).normalize();
        //Third row
        v1 = my_matrix.block(1,0,1,3).transpose();
        v2 = my_matrix.block(2,0,1,3).transpose();
            my_matrix.block(2,0,1,3) -= my_matrix.block(0,0,1,3) * (v0.dot(v2));
            my_matrix.block(2,0,1,3) -= my_matrix.block(1,0,1,3) * (v1.dot(v2));
            my_matrix.block(2,0,1,3).normalize();
            // check for positive determinant <=> right handed coordinate system of row vectors
        v2 = my_matrix.block(2,0,1,3).transpose();
            assert( (v0.cross(v1)).dot(v2) > 0 );
	}

	/// Exponentiate a vector in the Lie algebra to generate a new mySO3.
	/// See the Detailed Description for details of this vector.
    inline static mySO3 exp(const Eigen::Vector3d &vect);
	
	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	inline Eigen::Vector3d ln() const;

	/// Returns the inverse of this matrix (=the transpose, so this is a fast operation)
	mySO3 inverse() const {
        mySO3 result;

        result.get_matrix() = this->get_matrix().transpose();
        return result;
    }
	/// Returns the mySO3 as a Matrix<3>
    const Eigen::Matrix3d & get_matrix() const {return my_matrix;}
    Eigen::Matrix3d & get_matrix() {return my_matrix;}

  /// Returns the i-th generator times pos
  inline static Eigen::Vector3d generator_field(int i, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d result;
    result[i]=0;
    result[(i+1)%3] = - pos[(i+2)%3];
    result[(i+2)%3] = pos[(i+1)%3];
    return result;
  }

    Eigen::Matrix3d my_matrix;
};

/// Write an mySO3 to a stream 
/// @relates mySO3
inline std::ostream& operator<< (std::ostream& os, const mySO3& rhs){
	return os << rhs.get_matrix();
}
///Compute a rotation exponential using the Rodrigues Formula.
///The rotation axis is given by \f$\vec{w}\f$, and the rotation angle must
///be computed using \f$ \theta = |\vec{w}|\f$. This is provided as a separate
///function primarily to allow fast and rough matrix exponentials using fast 
///and rough approximations to \e A and \e B.
///
///@param w Vector about which to rotate.
///@param A \f$\frac{\sin \theta}{\theta}\f$
///@param B \f$\frac{1 - \cos \theta}{\theta^2}\f$
///@param R Matrix to hold the return value.
///@relates mySO3
inline void rodrigues_so3_exp(const Eigen::Vector3d& w, const double A, const double B, Eigen::Matrix3d& R) {
  assert(w.rows() == 3);
	{
		const double wx2 = (double)w(0)*w(0);
		const double wy2 = (double)w(1)*w(1);
		const double wz2 = (double)w(2)*w(2);
	
		R(0,0) = 1.0 - B*(wy2 + wz2);
		R(1,1) = 1.0 - B*(wx2 + wz2);
		R(2,2) = 1.0 - B*(wx2 + wy2);
	}
	{
		const double a = A*w(2);
		const double b = B*(w(0)*w(1));
		R(0,1) = b - a;
		R(1,0) = b + a;
	}
	{
		const double a = A*w(1);
		const double b = B*(w(0)*w(2));
		R(0,2) = b + a;
		R(2,0) = b - a;
	}
	{
		const double a = A*w(0);
		const double b = B*(w(1)*w(2));
		R(1,2) = b - a;
		R(2,1) = b + a;
	}
}


///Perform the exponential of the matrix \f$ \sum_i w_iG_i\f$
///@param w Weightings of the generator matrices.
inline mySO3 mySO3::exp(const Eigen::Vector3d& w){
	using std::sqrt;
	using std::sin;
	using std::cos;
    assert(w.rows() == 3);
	
	static const double one_6th = 1.0/6.0;
	static const double one_20th = 1.0/20.0;
	
	mySO3 result;
	
	const double theta_sq = w.dot(w);
	const double theta = sqrt(theta_sq);
	double A, B;
	//Use a Taylor series expansion near zero. This is required for
	//accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
	} else {
		if (theta_sq < 1e-6) {
			B = 0.5 - 0.25 * one_6th * theta_sq;
			A = 1.0 - theta_sq * one_6th*(1.0 - one_20th * theta_sq);
		} else {
			const double inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
		}
	}
	rodrigues_so3_exp(w, A, B, result.my_matrix);
	return result;
}

inline Eigen::Vector3d mySO3::ln() const{
	Eigen::Vector3d result;
	
	const double cos_angle = (my_matrix(0,0) + my_matrix(1,1) + my_matrix(2,2) - 1.0) * 0.5;
	result[0] = (my_matrix(2,1)-my_matrix(1,2))/2;
	result[1] = (my_matrix(0,2)-my_matrix(2,0))/2;
	result[2] = (my_matrix(1,0)-my_matrix(0,1))/2;
	
	double sin_angle_abs = sqrt(result.dot(result));
	if (cos_angle > M_SQRT1_2) {            // [0 - Pi/4[ use asin
		if(sin_angle_abs > 0){
			result *= asin(sin_angle_abs) / sin_angle_abs;
		}
	} else if( cos_angle > -M_SQRT1_2) {    // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
		const double angle = acos(cos_angle);
		result *= angle / sin_angle_abs;        
	} else {  // rest use symmetric part
		// antisymmetric part vanishes, but still large rotation, need information from symmetric part
		const double angle = M_PI - asin(sin_angle_abs);
		const double d0 = my_matrix(0,0) - cos_angle,
			d1 = my_matrix(1,1) - cos_angle,
			d2 = my_matrix(2,2) - cos_angle;
		Eigen::Vector3d r2;
		if(d0*d0 > d1*d1 && d0*d0 > d2*d2){ // first is largest, fill with first column
			r2[0] = d0;
			r2[1] = (my_matrix(1,0)+my_matrix(0,1))/2;
			r2[2] = (my_matrix(0,2)+my_matrix(2,0))/2;
		} else if(d1*d1 > d2*d2) { 			    // second is largest, fill with second column
			r2[0] = (my_matrix(1,0)+my_matrix(0,1))/2;
			r2[1] = d1;
			r2[2] = (my_matrix(2,1)+my_matrix(1,2))/2;
		} else {							    // third is largest, fill with third column
			r2[0] = (my_matrix(0,2)+my_matrix(2,0))/2;
			r2[1] = (my_matrix(2,1)+my_matrix(1,2))/2;
			r2[2] = d2;
		}
		// flip, if we point in the wrong direction!
		if(r2.dot(result) < 0)
			r2 *= -1;
		r2.normalize();
        result = angle*r2;
	} 
	return result;
}


/// Right-multiply by a Vector
/// @relates mySO3
inline Eigen::Vector3d operator*(const mySO3& lhs, const Eigen::Vector3d& rhs){
	return lhs.get_matrix() * rhs;
}

inline mySO3 operator*(const mySO3& lhs, const mySO3& rhs){

    mySO3 result;
    result.get_matrix() = lhs.get_matrix() * rhs.get_matrix();

    return result;
}

/// Represent a three-dimensional Euclidean transformation (a rotation and a translation). 
/// This can be represented by a \f$3\times\f$4 matrix operating on a homogeneous co-ordinate, 
/// so that a vector \f$\underline{x}\f$ is transformed to a new location \f$\underline{x}'\f$
/// by
/// \f[\begin{aligned}\underline{x}' &= E\times\underline{x}\\ \begin{bmatrix}x'\\y'\\z'\end{bmatrix} &= \begin{pmatrix}r_{11} & r_{12} & r_{13} & t_1\\r_{21} & r_{22} & r_{23} & t_2\\r_{31} & r_{32} & r_{33} & t_3\end{pmatrix}\begin{bmatrix}x\\y\\z\\1\end{bmatrix}\end{aligned}\f]
/// 
/// This transformation is a member of the Special Euclidean Lie group mySE3. These can be parameterised
/// six numbers (in the space of the Lie Algebra). In this class, the first three parameters are a
/// translation vector while the second three are a rotation vector, whose direction is the axis of rotation
/// and length the amount of rotation (in radians), as for mySO3
/// @ingroup gTransforms

class mySE3 {
public:
    /// Default constructor. Initialises the the rotation to zero (the identity) and the translation to zero
    inline mySE3() {
      my_translation.setZero();
    }

	/// Returns the rotation part of the transformation as a mySO3
	inline mySO3& get_rotation(){return my_rotation;}
	/// @overload
	inline const mySO3& get_rotation() const {return my_rotation;}

	/// Returns the translation part of the transformation as a Vector
	inline Eigen::Vector3d& get_translation() {return my_translation;}
	/// @overload
	inline const Eigen::Vector3d& get_translation() const {return my_translation;}

	/// Exponentiate a Vector in the Lie Algebra to generate a new mySE3.
	/// See the Detailed Description for details of this vector.
	/// @param vect The Vector to exponentiate
	static inline mySE3 exp(const Eigen::VectorXd& vect);

	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	static inline Eigen::VectorXd ln(const mySE3& se3);
    inline Eigen::VectorXd ln() const { return mySE3::ln(*this); }

    inline mySE3 inverse() const {
        mySE3 result;

        result.get_rotation() = this->get_rotation().inverse();
        result.get_translation() = -(result.get_rotation() * this->get_translation());

        return result;
	}

	/// Right-multiply by another mySE3 (concatenate the two transformations)
	/// @param rhs The multipier
	inline mySE3 operator *(const mySE3& rhs) const {
    mySE3 result;

    result.get_rotation().get_matrix() = this->get_rotation().get_matrix() * rhs.get_rotation().get_matrix();
    result.get_translation() = this->get_translation() + this->get_rotation().get_matrix() * rhs.get_translation();

    return result;
	}

  /// Returns the i-th generator times pos
  inline static Eigen::Vector4d generator_field(int i, const Eigen::Vector4d& pos) {
    Eigen::Vector4d result;
    result.setZero();
    if(i < 3) {
      result(i) = pos(3);
      return result;
    }
    result((i+1)%3) = -pos((i+2)%3);
    result((i+2)%3) = pos((i+1)%3);
    return result;
  }

private:
	mySO3 my_rotation;
	Eigen::Vector3d my_translation;
};

/// Write an mySE3 to a stream 
/// @relates mySE3
inline std::ostream& operator <<(std::ostream& os, const mySE3& rhs){
	std::streamsize fw = os.width();
	for(int i=0; i<3; i++){
		os.width(fw);
		os << rhs.get_rotation().get_matrix()(i,0) << " " << rhs.get_rotation().get_matrix()(i,1) << " " << rhs.get_rotation().get_matrix()(i,2);
		os.width(fw);
		os << " " << rhs.get_translation()(i) << '\n';
	}
	return os;
}

/// Left-multiply by a Matrix
/// @relates mySE3

inline mySE3 mySE3::exp(const Eigen::VectorXd& mu){
  assert(mu.rows() == 6);
	static const double one_6th = 1.0/6.0;
	static const double one_20th = 1.0/20.0;
	
	mySE3 result;
	
  Eigen::Vector3d w = mu.block(3,0,3,1);
	const double theta_sq = w.dot(w);
	const double theta = sqrt(theta_sq);
	double A, B;
	
  Eigen::Vector3d mu_3 = mu.block(0,0,3,1);
  Eigen::Vector3d cross = w.cross(mu_3);
	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
		result.get_translation() = mu.block(0,0,3,1) + 0.5 * cross;
	} else {
		double C;
		if (theta_sq < 1e-6) {
			C = one_6th*(1.0 - one_20th * theta_sq);
			A = 1.0 - theta_sq * C;
			B = 0.5 - 0.25 * one_6th * theta_sq;
		} else {
			const double inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
			C = (1 - A) * (inv_theta * inv_theta);
		}
		result.get_translation() = mu.block(0,0,3,1) + B*cross + C*w.cross(cross);
	}
	rodrigues_so3_exp(w, A, B, result.get_rotation().get_matrix());
	return result;
}

inline Eigen::VectorXd mySE3::ln(const mySE3& se3) {
	Eigen::Vector3d rot = se3.get_rotation().ln();
	const double theta = sqrt(rot.dot(rot));

	double shtot = 0.5;	
	if(theta > 0.00001) {
		shtot = sin(theta/2)/theta;
	}
	
	// now do the rotation
	const mySO3 halfrotator = mySO3::exp(rot * -0.5);
	Eigen::Vector3d rottrans = halfrotator * se3.get_translation();
	
	if(theta > 0.001){
		rottrans -= rot * ((se3.get_translation().dot(rot)) * (1-2*shtot) / (rot.dot(rot)));
	} else {
		rottrans -= rot * ((se3.get_translation().dot(rot))/24);
	}
	
	rottrans /= (2 * shtot);
	
	Eigen::VectorXd result(6);
    result(0) = rottrans(0);
    result(1) = rottrans(1);
    result(2) = rottrans(2);
    result(3) = rot(0);
    result(4) = rot(1);
    result(5) = rot(2);
	return result;
}

/// Right-multiply by a Vector
/// @relates mySE3
inline Eigen::Vector3d operator*(const mySE3& lhs, const Eigen::Vector3d& rhs) {
  Eigen::Vector3d result;

  result = lhs.get_translation() + lhs.get_rotation().get_matrix() * rhs;

  return result;
}

/// Right-multiply by a Matrix
/// @relates mySE3
inline Eigen::Matrix3d operator*(const mySE3& lhs, const Eigen::Matrix3d& rhs) {
  Eigen::Matrix3d result;
  Eigen::Vector3d vin, vout;

  for(int i=0;i<3;i++) {
    vin(0) = rhs(i,0);
    vin(1) = rhs(i,1);
    vin(2) = rhs(i,2);

    vout = lhs*vin;

    result(i,0) = vout(0);
    result(i,1) = vout(1);
    result(i,2) = vout(2);
  }

  return result;
}

class mySO2 {
public:
    /// Default constructor. Initialises the matrix to the identity (no rotation)
    mySO2() : my_matrix(Eigen::Matrix2d::Identity()) {}
    /// Returns the mySO3 as a Matrix<3>
    const Eigen::Matrix2d & get_matrix() const {return my_matrix;}
    Eigen::Matrix2d & get_matrix() {return my_matrix;}

    /// Assignment operator from a general matrix. This also calls coerce()
    /// to make sure that the matrix is a valid rotation matrix.
    mySO2& operator=(const Eigen::Matrix2d& rhs) {
        my_matrix = rhs;
        coerce();
        return *this;
    }

    /// Modifies the matrix to make sure it is a valid rotation matrix.
    void coerce(){
        Eigen::Vector2d v0, v1;

        //First row
        my_matrix.block(0,0,1,2).normalize();
        //Second row
        v0 = my_matrix.block(0,0,1,2).transpose();
        v1 = my_matrix.block(0,0,1,2).transpose();
        my_matrix.block(1,0,1,2) -= my_matrix.block(0,0,1,2) * (v0.dot(v1));
        my_matrix.block(1,0,1,2).normalize();


//        my_matrix[0] = unit(my_matrix[0]);
//        my_matrix[1] -= my_matrix[0] * (my_matrix[0]*my_matrix[1]);
//        my_matrix[1] = unit(my_matrix[1]);
    }

    /// Returns the inverse of this matrix (=the transpose, so this is a fast operation)
    mySO2 inverse() const {
        mySO2 result;

        result.get_matrix() = this->get_matrix().transpose();
        return result;
    }

    /// Exponentiate an angle in the Lie algebra to generate a new mySO2.
    inline static mySO2 exp(const double & d){
        mySO2 result;
        result.my_matrix(0, 0) = result.my_matrix(1, 1) = cos(d);
        result.my_matrix(1, 0) = sin(d);
        result.my_matrix(0, 1) = -result.my_matrix(1, 0);
        return result;
    }

private:
    Eigen::Matrix2d my_matrix;
};

/// Write an mySO3 to a stream
/// @relates mySO3
inline std::ostream& operator<< (std::ostream& os, const mySO2& rhs){
    return os << rhs.get_matrix();
}

/// Right-multiply by a Vector
/// @relates mySO3
inline Eigen::Vector2d operator*(const mySO2& lhs, const Eigen::Vector2d& rhs){
    return lhs.get_matrix() * rhs;
}

inline mySO2 operator*(const mySO2& lhs, const mySO2& rhs){

    mySO2 result;
    result.get_matrix() = lhs.get_matrix() * rhs.get_matrix();

    return result;
}

class mySE2{
public:
    /// Default constructor. Initialises the the rotation to zero (the identity) and the translation to zero
    mySE2() : my_translation(Eigen::Vector2d::Zero()) {}

    /// Returns the rotation part of the transformation as a mySO3
    inline mySO2& get_rotation(){return my_rotation;}
    /// @overload
    inline const mySO2& get_rotation() const {return my_rotation;}

    /// Returns the translation part of the transformation as a Vector
    inline Eigen::Vector2d& get_translation() {return my_translation;}
    /// @overload
    inline const Eigen::Vector2d& get_translation() const {return my_translation;}

    inline mySE2 inverse() const {
        mySE2 result;
        result.get_rotation() = this->get_rotation().inverse();
        result.get_translation() = -(result.get_rotation() * this->get_translation());

        return result;
    }

    /// Right-multiply by another mySE2 (concatenate the two transformations)
    /// @param rhs The multipier
    inline mySE2 operator *(const mySE2& rhs) const {
        mySE2 result;

        result.get_rotation().get_matrix() = this->get_rotation().get_matrix() * rhs.get_rotation().get_matrix();
        result.get_translation() = this->get_translation() + this->get_rotation().get_matrix() * rhs.get_translation();

        return result;
    }

private:
    Eigen::Vector2d my_translation;
    mySO2 my_rotation;
};

/// @relates mySE2
inline std::ostream& operator <<(std::ostream& os, const mySE2& rhs){
    std::streamsize fw = os.width();
    for(int i=0; i<2; i++){
        os.width(fw);
        os << rhs.get_rotation().get_matrix()(i,0) << " " << rhs.get_rotation().get_matrix()(i,1);
        os.width(fw);
        os << " " << rhs.get_translation()(i) << '\n';
    }
    return os;
}

/// Right-multiply by a Vector
/// @relates mySE2
inline Eigen::Vector2d operator*(const mySE2& lhs, const Eigen::Vector2d& rhs) {
  Eigen::Vector2d result;

  result = lhs.get_translation() + lhs.get_rotation().get_matrix() * rhs;

  return result;
}

/// Right-multiply by a Matrix
/// @relates mySE2
inline Eigen::Matrix2d operator*(const mySE2& lhs, const Eigen::Matrix2d& rhs) {
  Eigen::Matrix2d result;
  Eigen::Vector2d vin, vout;

  for(int i=0;i<2;i++) {
    vin(0) = rhs(i,0);
    vin(1) = rhs(i,1);

    vout = lhs*vin;

    result(i,0) = vout(0);
    result(i,1) = vout(1);
  }

  return result;
}

#endif
