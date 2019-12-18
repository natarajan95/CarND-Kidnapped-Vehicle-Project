/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;


//static default_random_engine gen;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 25;  // TODO: Set the number of particles
  // Create normal distributions for x, y and theta
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);
  
  for (int i = 0; i < num_particles; ++i) {
    
    Particle p;
    
    p.id = i;
    p.x = x + dist_x(gen);
    p.y = y + dist_y(gen);
    p.theta = theta + dist_theta(gen); 
    p.weight = 1;
     
    weights.push_back(1);
    particles.push_back(p);
  }

  is_initialized = true;  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
    std::default_random_engine gen;
 	// Create Gaussian noise distributions
    std::normal_distribution<double> noise_x(0, std_pos[0]);
    std::normal_distribution<double> noise_y(0, std_pos[1]);
    std::normal_distribution<double> noise_theta(0, std_pos[2]);

	for (unsigned int i = 0; i < particles.size(); ++i) {

    double x_pred = 0.0;
    double y_pred = 0.0;
    double theta_pred = 0.0;
	// Prediction formula from Module 2 - Lesson 3 - Yaw rate & Velocity
	if (fabs(yaw_rate) < 0.00001) {
      x_pred = particles[i].x + velocity * cos(particles[i].theta) * delta_t;
      y_pred = particles[i].y + velocity * sin(particles[i].theta) * delta_t;
      theta_pred = particles[i].theta + yaw_rate * delta_t;
    } else {
      x_pred = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      y_pred = particles[i].y + (velocity / yaw_rate) * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
      theta_pred = particles[i].theta + yaw_rate * delta_t;
    }

	// Noise + Updated x,y,theta
    particles[i].x = x_pred + noise_x(gen);
    particles[i].y = y_pred + noise_y(gen);
    particles[i].theta = theta_pred + noise_theta(gen);


}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (unsigned int i=0; i< observations.size(); ++i){
    //double dist_obs = dist(observations[i].x, observations[i].y, observations[i].theta)
    double min_dist = 10000000; //Arbt big value
	int temp_id = -1;
    for (unsigned int j=0; j< predicted.size(); ++j){
      // Find distance between nObservation & pred
      double dist_btwn = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      // Check if this is the closest
      if (dist_btwn < min_dist){
        min_dist = dist_btwn;
        temp_id = j; //assign the observed measurement to this particular landmark
        }
    }
	observations[i].id = predicted[temp_id].id;
    
    
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   weights.clear();
   std::default_random_engine gen;
  for (int i = 0; i < num_particles; ++i){
    double p_x = particles[i].x;
    double p_y = particles[i].y;
	double p_t = particles[i].theta;
    //double p_id = particles[i].id;
 
	
	// Observations - Transform from Veh to Map coordinates and store in a vector
	vector<LandmarkObs> obs_mapCoord;
	for (unsigned int k = 0; k < observations.size(); ++k) {
		if (dist(observations[k].x, observations[k].y, 0, 0) <= sensor_range) {
	  int map_id = -1;
      double map_x = p_x + observations[k].x * cos(p_t) -  observations[k].y * sin(p_t);
      double map_y = p_y + observations[k].x * sin(p_t) +  observations[k].y * cos(p_t);
	  obs_mapCoord.push_back(LandmarkObs{map_id, map_x, map_y});
		}
	}




 
    vector<LandmarkObs> pred_within_range;
    // Iterate to find landmarks in particle range
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size();  ++j){
      // Get x,y, if from map_landmarks
      double l_map_x = map_landmarks.landmark_list[j].x_f;
      double l_map_y = map_landmarks.landmark_list[j].y_f;
      int l_map_id = map_landmarks.landmark_list[j].id_i;
      // Dist between particle & landmark in map coordinate
      // if (fabs(l_map_x - p_x) <= sensor_range && fabs(l_map_y - p_y) <= sensor_range) {
      // Check if within sensor range
	  if (dist(p_x, p_y, l_map_x, l_map_y) <= sensor_range){
        pred_within_range.push_back(LandmarkObs{l_map_id, l_map_x, l_map_y});
      }
    }


	//dataAssociation(predicted_landmarks, transformed_observations);
	dataAssociation(pred_within_range, obs_mapCoord);
    //double new_wt = 1.0;
	particles[i].weight = 1.0;

	////////////////////////////   
	
    double weight = 1.0;	
	for (unsigned int j = 0; j < pred_within_range.size(); j++){
      double min_dist = 1000000; //Arbt big value
	  int min_k = -1;  
      for (unsigned int k = 0; k < obs_mapCoord.size(); ++k){
	    if (pred_within_range[j].id == obs_mapCoord[k].id){
					if (dist(pred_within_range[j].x, pred_within_range[j].y, obs_mapCoord[k].x, obs_mapCoord[k].y) < min_dist) {
						min_dist = dist(pred_within_range[j].x, pred_within_range[j].y, obs_mapCoord[k].x, obs_mapCoord[k].y);
						min_k = k;
					}
		  
		}
	  }
	  
			if (min_k != -1) {
					  double f1 = 1.0 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	  double f2 = pow ( (obs_mapCoord[min_k].x - pred_within_range[j].x) , 2) / (2 * pow( std_landmark[0], 2) );
	  double f3 = pow ( (obs_mapCoord[min_k].y - pred_within_range[j].y) , 2) / (2 * pow( std_landmark[1], 2) );
	  weight = weight * f1 * exp(-(f2+ f3));
      }
	}
	
	  
		
		weights.push_back(weight);
		particles[i].weight = weight;

  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight. 
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  std::default_random_engine gen; 
  vector<Particle> new_particles;

  // get all of the current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  // generate random starting index for resampling wheel
  std::uniform_int_distribution<int> dist_for_index(0, num_particles-1);
  int index = dist_for_index(gen);

  // get max weight
  double max_weight = *max_element(weights.begin(), weights.end());

  // uniform random distribution [0.0, max_weight)
  std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

  double beta = 0.0;

  // spin the resample wheel!
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}