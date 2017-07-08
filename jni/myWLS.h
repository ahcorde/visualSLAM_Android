#ifndef MYWLS_H
#define MYWLS_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <vector>
#include <cmath>
#include "/Users/ahcorde/Downloads/eigen/Eigen/Dense"

template <int Size>
class myWLS {
public:
  myWLS(int size=Size) {
    my_C_inv.resize(size, size);
    my_vector.resize(size,1);
    my_mu.resize(size);

		clear();
	}

	/// Clear all the measurements and apply a constant regularisation term. 
	void clear() {
    my_C_inv.setZero();
		my_vector.setZero();
	}

	/// Applies a constant regularisation term. 
	/// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
	/// @param val The strength of the prior
	void add_prior(double val){
		for(int i=0; i<my_C_inv.rows(); i++){
      my_C_inv(i,i)+=val;
		}
	}
  
	/// Add a single measurement 
	/// @param m The value of the measurement
	/// @param J The Jacobian for the measurement \f$\frac{\partial\text{m}}{\partial\text{param}_i}\f$
	/// @param weight The inverse variance of the measurement (default = 1)
	inline void add_mJ(double m, Eigen::MatrixXd& J, double weight = 1) {
		
		//Upper right triangle only, for speed
		for(int r=0; r < my_C_inv.rows(); r++) {
			double Jw = weight * J(r);
			my_vector(r) += m * Jw;
			for(int c=r; c < my_C_inv.rows(); c++) {
				my_C_inv(r,c) += Jw * J(c);
      }
		}
	}

	/// Process all the measurements and compute the weighted least squares set of parameter values
	/// stores the result internally which can then be accessed by calling get_mu()
	void compute(){
	
		//Copy the upper right triangle to the empty lower-left.
		for(int r=1; r < my_C_inv.rows(); r++)
			for(int c=0; c < r; c++) {
        my_C_inv(r,c) = my_C_inv(c,r);
      }

      my_mu = my_C_inv.inverse()*my_vector;
	}

  ///<Returns the update. With no prior, this is the result of \f$J^\dagger e\f$.
  Eigen::VectorXd get_mu() {
    return my_mu;  
  }

private:
	Eigen::MatrixXd my_C_inv;
	Eigen::MatrixXd my_vector;
	Eigen::VectorXd my_mu;
};

#endif

