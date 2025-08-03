#include "pollution_detector.h"

PollutionDetector::PollutionDetector(float iaqThreshold, float vocThreshold, float co2Threshold, float pm25Threshold)
    : _iaqThreshold(iaqThreshold), _vocThreshold(vocThreshold), 
      _co2Threshold(co2Threshold), _pm25Threshold(pm25Threshold) {}

PollutionDetector::DetectionResult PollutionDetector::detect(float iaq, float voc, float co2, float temp, float humidity, bool inSpike) {
    DetectionResult result;
    result.isThreat = false;
    result.isSpike = inSpike;

    // Early exit for clean air conditions
    if (!inSpike && iaq < 50 && voc < 0.5f && co2 < 800) {
        result.signature = "Clean_Air";
        return result;
    }

    int highestPriority = 99;
    int matchCount = 0;

    for (int i = 0; i < PollutionSignatures::getNumSignatures(); i++) {
        const PollutionPattern& pattern = PollutionSignatures::getSignatures()[i];
        
        bool iaqMatch = (iaq >= pattern.minIAQ) && (iaq <= pattern.maxIAQ);
        bool vocMatch = (voc >= pattern.minVOC) && (voc <= pattern.maxVOC);
        bool co2Match = (co2 >= pattern.minCO2) && (co2 <= pattern.maxCO2);
        bool tempMatch = (temp >= pattern.minTemp) && (temp <= pattern.maxTemp);
        bool humidityMatch = (humidity >= 20 && humidity <= 60); // Common attack conditions

        int matches = iaqMatch + vocMatch + co2Match + tempMatch;

        // For threat signatures, require humidity match and active spike
        if (pattern.isThreat) {
            matches += humidityMatch;
            if (!inSpike) continue;
        }

        if (matches >= 3) {
            if (pattern.priority < highestPriority) {
                highestPriority = pattern.priority;
                result.signature = pattern.name;
                result.isThreat = pattern.isThreat;
                matchCount = 1;
            }
            else if (pattern.priority == highestPriority) {
                if (matchCount > 0) {
                    result.signature += "+";
                }
                result.signature += pattern.name;
                matchCount++;
            }
        }
    }

   // Mmain detection logic:
if (result.signature.isEmpty()) {
    // Clean air conditions (unified logic)
    if (iaq <= 50 && voc <= 0.6f && co2 <= 600) {
        if (iaq <= 25 && voc <= 0.3f) {
            result.signature = "Pristine_Air_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 2) + "ppm";
        } else {
            result.signature = "Clean_Air_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 2) + "ppm";
        }
    }
    // Moderate air quality
    else if (iaq <= 100 && voc <= 1.0f && co2 <= 800) {
        result.signature = "Moderate_Air_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 2) + "ppm";
    }
    // Pollution detection thresholds
    else if (voc > 5.0f && inSpike) {
        result.signature = "Suspicious_VOC_Spike_" + String(voc, 1) + "ppm";
    }
    else if (voc > 2.0f) {
        result.signature = "High_VOC_" + String(voc, 1) + "ppm";
    }
    else if (co2 > 1000) {
        result.signature = "CO2_Hazard_" + String(co2, 0) + "ppm";
    }
    else if (iaq > 250) {
        result.signature = "Severe_Pollution_IAQ" + String(iaq, 0);
    }
    else if (iaq > 150) {
        result.signature = "Heavy_Pollution_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 1) + "ppm";
    }
    else if (iaq > 100) {
        result.signature = "Unhealthy_Air_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 1) + "ppm";
    }
    else {
        result.signature = "Unknown_Source_IAQ" + String(iaq, 0) + "_VOC" + String(voc, 2) + "ppm";
    }
}

    // Add threat indicator for high priority signatures
    if (highestPriority <= 2) {
        result.isThreat = true;
    }

    return result;
}

bool PollutionDetector::isSpike(float currentValue, float baselineValue, float threshold) const {
    return (currentValue - baselineValue) > threshold;
}

void PollutionDetector::setThresholds(float iaq, float voc, float co2, float pm25) {
    _iaqThreshold = iaq;
    _vocThreshold = voc;
    _co2Threshold = co2;
    _pm25Threshold = pm25;
}