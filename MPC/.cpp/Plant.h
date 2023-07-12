// File: Plant.h
// Created by: Michael Napoli
// Created on: Jul 07, 2023


#include<iostream>
#include<functional>

// check if the Plant.h has already been included
#ifndef PLANT
#define PLANT

namespace napolean
{
    class State:
    {
    private:
        int n;
        int m;

    public:
        double* x;

        // Member constructors.
        State(const int &n);
        State(const int &n, const double &x);

        // Accessor Functions
        // NONE

        // Destructor
        ~State();
    };
}