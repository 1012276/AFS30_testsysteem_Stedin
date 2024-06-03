#include <iostream>
#include <cmath>
#include <iostream>
#include <cmath>
const double PI = 3.14159265358979323846;

double calculateAmplitude(double userAmplitude) {
    if (userAmplitude < 5 || userAmplitude > 500) {
        std::cout << "Warning: Amplitude should be between 5 and 500." << std::endl;
        return 0.0;
    }
    return userAmplitude / 10000.0;
}

double calculateFrequency(int harmonic) {
    return harmonic * 50.0;
}

double calculateSignal(double amplitude, double frequency, double time) {
    return amplitude * sin(2 * PI * frequency * time);
}

int main() {
    double userAmplitude;
    std::cout << "Enter the amplitude (between 5 and 500): ";
    std::cin >> userAmplitude;

    double amplitude = calculateAmplitude(userAmplitude);
    if (amplitude == 0.0) {
        return 0;
    }

    int maxHarmonic;
    std::cout << "Do you want to set amplitudes for odd harmonics? (yes/no): ";
    std::string choice;
    std::cin >> choice;

    if (choice == "yes") {
        std::cout << "Enter the maximum harmonic (up to 13): ";
        std::cin >> maxHarmonic;

        if (maxHarmonic < 1 || maxHarmonic > 13 || maxHarmonic % 2 == 0) {
            std::cout << "Invalid maximum harmonic. Please enter an odd number between 1 and 13." << std::endl;
            return 0;
        }

        double percentage;
        if (maxHarmonic == 13) {
            std::cout << "Enter the percentage of all odd harmonics (5-50%): ";
            std::cin >> percentage;

            if (percentage < 5 || percentage > 50 || fmod(percentage, 1.0) != 0.0) {
                std::cout << "Invalid percentage. Please enter an integer percentage between 5 and 50." << std::endl;
                return 0;
            }
        }

        double totalAmplitude = amplitude;
        for (int harmonic = 3; harmonic <= maxHarmonic; harmonic += 2) {
            double harmonicAmplitude;
            if (harmonic == 13) {
                harmonicAmplitude = (percentage / 100.0) * totalAmplitude;
            } else {
                std::cout << "Enter the amplitude for harmonic " << harmonic << ": ";
                std::cin >> harmonicAmplitude;
            }

            totalAmplitude += harmonicAmplitude;
        }

        std::cout << "Total Harmonic Distortion (THD%): " << ((totalAmplitude - amplitude) / amplitude) * 100.0 << std::endl;
    }

    double timeStep = 1.0 / 44100.0; // Assuming a sample rate of 44.1 kHz
    double totalTime = 1.0; // Total time for which the signal will be generated

    for (int harmonic = 1; harmonic <= 13; harmonic += 2) {
        double frequency = calculateFrequency(harmonic);
        for (double t = 0.0; t < totalTime; t += timeStep) {
            double signal = calculateSignal(amplitude, frequency, t);
            // Output the signal to the DAC or perform further processing
            std::cout << signal << std::endl;
        }
    }

    return 0;
}