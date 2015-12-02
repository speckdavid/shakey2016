#ifndef SHAKEY_CLASSIFIER_H_
#define SHAKEY_CLASSIFIER_H_

#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>

#define GROUND_UPPER_BOUND -0.1
#define ARC_THRESH 5

enum Object_class {Box, Wedge, Ground, Unkown};

class Classifier
{
public:
  Classifier() {}
  
  Object_class classify(std::vector<pcl::ModelCoefficients> coeffs)
  {
	// Check if only one plane is found -> Ground
	if (coeffs.size() < 2)
	  return Ground;
	for (int i = 0; i < coeffs.size(); i++) {
	  pcl::ModelCoefficients cur_coeff = coeffs.at(i);
	  // Round coeffs
	  for (int j = 0; j < 4; j++) {
		cur_coeff.values[j] = roundf(cur_coeff.values[j]*100)/100;
	  }
	  // Check for plane parallel to ground plane -> Box
	  if (std::abs(cur_coeff.values[0]) < 0.05 && std::abs(cur_coeff.values[1]) < 0.05 
	    && std::abs(cur_coeff.values[2] - 1) < 0.05  && cur_coeff.values[3] < GROUND_UPPER_BOUND)
	    return Box;
    }
    return Wedge;
  }	
};

#endif  // SHAKEY_CLASSIFIER_H_
