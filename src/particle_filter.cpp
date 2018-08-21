/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <list>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Creates a normal (Gaussian) distribution for x
  default_random_engine gen;
	normal_distribution<double> dist_x(0, std[0]);
	normal_distribution<double> dist_y(0, std[1]);
	normal_distribution<double> dist_theta(0, std[2]);
  
	num_particles = 200; // 400 works too;   
  for (int i = 0; i < num_particles; ++i) { 
    Particle p = Particle();
    p.id = i;
    p.x = x + dist_x(gen);
    p.y = y + dist_y(gen);
    p.theta = theta + dist_theta(gen);
    p.weight = 1.0;
    p.associations.clear(); 
    p.sense_x.clear();
    p.sense_y.clear();
    
    particles.push_back(p);  
    weights.push_back(1.0);
  }
  
  is_initialized = true;
   
  return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
  
  for (int i = 0; i < num_particles; ++i) { 
    // Update the particle's position and orientation.
    double theta = particles[i].theta;
    if (yaw_rate == 0) {  // account for when yaw_rate == 0
      double theta_0 = theta;
      particles[i].x = particles[i].x + velocity*delta_t*cos(theta_0);
      particles[i].y = particles[i].y + velocity*delta_t*sin(theta_0);
      theta = theta_0;
    }
    else { 
      double theta_0 = theta;
      double theta_f = theta_0 + yaw_rate*delta_t;
      particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(theta_f) - sin(theta_0));
      particles[i].y = particles[i].y + (velocity/yaw_rate)*(cos(theta_0) - cos(theta_f));
      theta = theta_f;
    }
    // add noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    
    theta += dist_theta(gen);
    particles[i].theta = theta;
  }
  
  return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  // Did not need to implement this function.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  double g_norm = 1./sqrt(2*M_PI*std_x*std_y);
  double g_inv_x = 1./(2*std_x*std_x);
  double g_inv_y = 1./(2*std_y*std_y);
  
  for (int i = 0; i < num_particles; ++i) { 
    
    particles[i].associations.clear();
    particles[i].sense_x.clear();
    particles[i].sense_y.clear();
    
    double x_p = particles[i].x;
    double y_p = particles[i].y;
    double theta_p = particles[i].theta;

    double wt = 1;
    for (size_t j=0; j<observations.size(); ++j) {
      // transform between the vehicle's coordinate system and the map's coordinate system. 
      double x_o = cos(theta_p)*observations[j].x - sin(theta_p)*observations[j].y + x_p;
      double y_o = sin(theta_p)*observations[j].x + cos(theta_p)*observations[j].y + y_p;
    
      // find the closest landmark to each observation (nearest neighbor approach).
      int k_min = 0;  
      double x_m = map_landmarks.landmark_list[0].x_f;
      double y_m = map_landmarks.landmark_list[0].y_f;
      double min_dist = dist(x_o,y_o,x_m,y_m);
      for (size_t k = 1; k < map_landmarks.landmark_list.size(); ++k) {
        x_m = map_landmarks.landmark_list[k].x_f;
        y_m = map_landmarks.landmark_list[k].y_f;        
        double min_dist_k = dist(x_o,y_o,x_m,y_m);
        if (min_dist_k < min_dist) {
          k_min = k;
          min_dist = min_dist_k;
        }
      }
      // associate the landmark found above with the particle.
      particles[i].associations.push_back(map_landmarks.landmark_list[k_min].id_i);
      particles[i].sense_x.push_back(map_landmarks.landmark_list[k_min].x_f);
      particles[i].sense_y.push_back(map_landmarks.landmark_list[k_min].y_f);
            
      x_m = map_landmarks.landmark_list[k_min].x_f;
      y_m = map_landmarks.landmark_list[k_min].y_f;   

      // calculate the weight of the measurement using a multivariate Gaussian probability density.
      double wt_j = g_norm*exp(-g_inv_x*(x_o-x_m)*(x_o-x_m) -g_inv_y*(y_o-y_m)*(y_o-y_m));

      // multiply the weights to get the particle's weight.
      wt *= wt_j;
    }
    particles[i].weight = wt;
    weights[i] = wt;
  }
  
  return;
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(),weights.end());
  
  std::vector<Particle> new_particles;
  for (int i=0; i<num_particles; ++i) { 
    int n = d(gen);
    Particle p = particles[n];
    new_particles.push_back(p);
  }
  
  particles = new_particles;
  
  return;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    Particle p = particle;
    p.associations = associations;
    p.sense_x = sense_x;
    p.sense_y = sense_y;
    
    return p;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
