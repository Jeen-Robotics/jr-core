#include <chrono>
#include <thread>

#include <jr_math/jr_math.h>

// Basic arithmetic implementations
double add(double a, double b) {
  return a + b;
}

double subtract(double a, double b) {
  return a - b;
}

double multiply(double a, double b) {
  return a * b;
}

double divide(double a, double b) {
  if (b == 0) {
    return 0; // In a real application, you might want to handle this
              // differently
  }
  return a / b;
}

// Async implementation
void async_add(double a, double b, AsyncCallback callback) {
  // Simulate a long-running operation
  std::thread([a, b, callback]() {
    // Simulate some processing time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    double result = a + b;
    callback(result);
  }).detach();
}