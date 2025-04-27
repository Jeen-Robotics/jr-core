#ifndef ARITHMETIC_H
#define ARITHMETIC_H

#ifdef __cplusplus
extern "C" {
#endif

// Basic arithmetic functions
double add(double a, double b);
double subtract(double a, double b);
double multiply(double a, double b);
double divide(double a, double b);

// Async operation callback type
typedef void (*AsyncCallback)(double result);

// Async arithmetic function
void async_add(double a, double b, AsyncCallback callback);

#ifdef __cplusplus
}
#endif

#endif // ARITHMETIC_H