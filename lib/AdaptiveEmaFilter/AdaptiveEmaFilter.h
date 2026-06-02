#include <cmath> // Nodig voor std::fabs

class AdaptiveEmaFilter {
private:
    float _filtered_val;
    float _diff_smooth;
    bool _is_initialized;

    // Tuning parameters
    float _alpha_base;   // Basis filtersterkte bij stilstand (ruisonderdrukking)
    float _sensitivity;  // Gevoeligheid voor snelle veranderingen

public:
    // Constructor: stel hier de tuningwaarden in
    AdaptiveEmaFilter(float base_filter = 0.01f, float sens = 0.1f) 
        : _filtered_val(0.0f), _diff_smooth(0.0f), _is_initialized(false),
          _alpha_base(base_filter), _sensitivity(sens) {}

    // Reset functie (handig bij het opstarten van het systeem)
    void reset() {
        _is_initialized = false;
        _diff_smooth = 0.0f;
    }

    void setBaseFilter(float base_filter) {
        _alpha_base = base_filter;
    }

    float getBaseFilter() const {
        return _alpha_base;
    }

    void setSensitivity(float sens) {
        _sensitivity = sens;
    }

    float getSensitivity() const {
        return _sensitivity;
    }
    // Het eigenlijke filteralgoritme (elke 20ms aanroepen)
    float update(float raw_input) {
        if (!_is_initialized) {
            _filtered_val = raw_input;
            _diff_smooth = 0.0f;
            _is_initialized = true;
            return _filtered_val;
        }

        // 1. Bereken het verschil tussen de rauwe meting en de huidige gefilterde waarde
        float diff = raw_input - _filtered_val;

        // 2. Filter deze fout om kortstondige ruispieken uit te dempen
        // (0.025 en 0.975 bepalen hoe snel het filter reageert op een trendbreuk)
        _diff_smooth = (0.025f * diff) + (0.975f * _diff_smooth);

        // 3. Bereken de dynamische alpha (begrensd tussen alpha_base en 1.0)
        // float alpha = _alpha_base + (_sensitivity * std::fabs(_diff_smooth));
        float _diff_scaled = _diff_smooth * _sensitivity; // Schaal de fout met de gevoeligheid
        // Gebruik het kwadraat van de geschaalde fout om een sterker effect te hebben bij grotere fouten en een milder effect bij kleinere fouten.
        // Verwijder tevens de richting van de fout, zodat we zowel bij positieve als negatieve fouten een hogere alpha krijgen.
        // Begrens vervolgens de alpha tot een maximum van 1.0.
        float alpha = _alpha_base + (_diff_scaled * _diff_scaled);
        if (alpha > 1.0f) {
            alpha = 1.0f;
        }

        // 4. Bereken de nieuwe gefilterde waarde
        _filtered_val = (alpha * raw_input) + ((1.0f - alpha) * _filtered_val);

        return _filtered_val;
    }
};
