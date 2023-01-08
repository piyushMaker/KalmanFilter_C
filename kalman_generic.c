#include <stdio.h>
#include <math.h>

#define DT 0.01  // time step
#define A 3  // number of dimensions (acceleration has 3 dimensions)

// state vector
double x[A] = {0};

// covariance matrix
double P[A][A] = {{1, 0}, {0, 1}};

// process noise covariance matrix
double Q[A][A] = {{0.1, 0}, {0, 0.1}};

// measurement noise covariance matrix
double R[A][A] = {{1, 0}, {0, 1}};

// Kalman gain
double K[A] = {0};

// measurement vector
double z[A] = {0};

// accelerometer measurement
double accelerometer[A] = {0};

void predict() {
  // predict state vector
  for (int i = 0; i < A; i++) {
    x[i] = x[i] + DT * x[i];
  }

  // predict covariance matrix
  for (int i = 0; i < A; i++) {
    for (int j = 0; j < A; j++) {
      P[i][j] = P[i][j] + DT * (P[i][j] + P[j][i] + Q[i][j]);
    }
  }
}

void update() {
  // calculate Kalman gain
  for (int i = 0; i < A; i++) {
    K[i] = P[i][i] / (P[i][i] + R[i][i]);
  }

  // update state vector
  for (int i = 0; i < A; i++) {
    x[i] = x[i] + K[i] * (z[i] - x[i]);
  }

  // update covariance matrix
  for (int i = 0; i < A; i++) {
    for (int j = 0; j < A; j++) {
      P[i][j] = P[i][j] - K[i] * P[i][j];
    }
  }
}

int main() {
  // set initial state vector and covariance matrix
  for (int i = 0; i < A; i++) {
    x[i] = 0;
    for (int j = 0; j < A; j++) {
      P[i][j] = 0;
    }
  }

  // read accelerometer measurements
  for (int i = 0; i < A; i++) {
    scanf("%lf", &accelerometer[i]);
  }

  // set measurement vector
  for (int i = 0; i < A; i++) {
    z[i] = accelerometer[i];
  }

  // run Kalman filter
  for (int i = 0; i < 100; i++) {
    predict();
    update();
  }

  // print smoothed accelerometer values
  printf("Smoothed accelerometer values:\n");
  for (int i = 0; i < A; i++) {
    printf("%lf\n", x[i]);
    //printf("%.3lf\n", x[i]); //prints upto 3 decimal values
  }
