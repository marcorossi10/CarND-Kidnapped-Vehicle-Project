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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  //std::vector<Particle> particles{};
  
  // Create a normal distribution for x,y,theta with the associated variances
  std::default_random_engine gen;
  std::normal_distribution<double> distribution_x(x, std[0]);
  std::normal_distribution<double> distribution_y(y, std[1]);
  std::normal_distribution<double> distribution_theta(theta, std[2]);

  //Create a vector of empty particles
  for(int i=0; i<num_particles; i++)
  {
    Particle particle{};
    particles.emplace_back(particle);
  }

  // Set all the particles to initialization values sampling from the gaussian distribution
  for(int i=0; i<num_particles; i++)
  {
    particles.at(i).id = i;
    particles.at(i).x = distribution_x(gen);
    particles.at(i).y = distribution_y(gen);
    particles.at(i).theta = distribution_theta(gen);
    particles.at(i).weight = 1.0;
    weights.emplace_back(particles.at(i).weight);
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
  
  // Create a normal distribution for x,y,theta with the associated variances
  std::default_random_engine gen;
  std::normal_distribution<double> distribution_x(0.0, std_pos[0]);
  std::normal_distribution<double> distribution_y(0.0, std_pos[1]);
  std::normal_distribution<double> distribution_theta(0.0, std_pos[2]);

  //Implement bycicle model prediction for each particle
  for(int i=0; i<num_particles; i++)
  {
    if (yaw_rate < 0.00001) //this if-else is to handle also cases where the particle has constant yaw (i.e. zero yaw rate)
    {
      particles.at(i).x = particles.at(i).x + velocity*delta_t*cos(particles.at(i).theta) + distribution_x(gen);
      particles.at(i).y = particles.at(i).y + velocity*delta_t*sin(particles.at(i).theta) + distribution_y(gen);
      particles.at(i).theta = particles.at(i).theta + distribution_theta(gen);
    }
    else 
    {
      particles.at(i).x = particles.at(i).x + (velocity/yaw_rate)*(sin(particles.at(i).theta+(yaw_rate*delta_t)) - sin(particles.at(i).theta)) + distribution_x(gen);
      particles.at(i).y = particles.at(i).y + (velocity/yaw_rate)*(cos(particles.at(i).theta) - cos(particles.at(i).theta+(yaw_rate*delta_t))) + distribution_y(gen);
      particles.at(i).theta = particles.at(i).theta + yaw_rate*delta_t + distribution_theta(gen);
    }
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> landmarks, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the landmark that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < observations.size(); i++)
  {
    for (int j = 0; j < predicted_landmarks.size(); j++)
    {
      //current_dist = dist(observations[i].x, observations[i].y, predicted_landmarks[i].x, predicted_landmarks[i].y);
    }

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

  for (int i = 0; i < num_particles; i++)
  {
    //Transform measurements in map coordinate system (homogeneous transformation) for each particle
    vector<LandmarkObs> observations_in_map_ref_system{};
    for  (int j = 0; j < observations.size(); j++)
    {
      LandmarkObs current_map_obs;
      current_map_obs.id = observations[j].id;
      current_map_obs.x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      current_map_obs.y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      observations_in_map_ref_system.emplace_back(current_map_obs);
    }

    //Find the landmarks (on map) in the sensor range for each particle:
    //The distance between particle and landmark has to be smaller than sensor_range
    vector<LandmarkObs> seen_landmarks{};
    for  (int j = 0; j < map_landmarks.landmark_list.size(); j++)
    {
      double particle2landmark_dist{dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f)};
      if (particle2landmark_dist < sensor_range)
      {
        seen_landmarks.emplace_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f});
      }
    }

    //Data association
  }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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