#include <stdio.h>
#include <stdbool.h>
#include "inverter_model.h"

int main() {
    // 1. Setup Parameters
    InverterParams p = {
        .Rdson = 0.05f,   // 50 mOhm
        .RdsOff = 1e6f,   // 1 MOhm (High-Z)
        .FWmin = 0.01f    // 10mA threshold for diode conduction
    };

    InverterInput in = {
        .vbus = 24.0f,
        .duty = {0.5f, 0.5f, 0.5f},  // 50% duty
        .drive = {1, 0, 0}           // Phase A active, others inactive
    };

    InverterState state = {0};
    InverterOutput out = {0};

    // 2. Open CSV for results
    FILE *f = fopen("inverter_test.csv", "w");
    if (!f) return 1;
    fprintf(f, "current_ia,drive_a,vthev_a,rthev_a,mode\n");

    printf("Testing Inverter Model...\n");

    // 3. Test Sweep: Varying current from -2A to 2A
    // We will test with Drive ON then Drive OFF
    for (int drive_state = 1; drive_state >= 0; drive_state--) {
        in.drive[0] = (float)drive_state;

        for (float current = -2.0f; current <= 2.0f; current += 0.1f) {
            state.i[0] = current;
            
            inverter_step(&state, &in, &p, &out);

            // Determine mode for easier CSV reading
            const char* mode = "High-Z";
            if (drive_state == 1) mode = "PWM_Active";
            else if (current < -p.FWmin) mode = "Diode_Top";
            else if (current > p.FWmin) mode = "Diode_Bottom";

            fprintf(f, "%.2f,%d,%.4f,%.4f,%s\n", 
                    current, drive_state, out.vthev[0], out.rthev[0], mode);
        }
    }

    fclose(f);
    printf("Done. Results saved to inverter_test.csv\n");

    // 4. Verification Check (Sanity Check in Terminal)
    // Test Case: Drive OFF, Current Pushing into Top Diode
    in.drive[0] = 0;
    state.i[0] = -1.0f; 
    inverter_step(&state, &in, &p, &out);
    printf("\nSanity Check (Phase A):");
    printf("\nDrive: 0, Current: -1.0A -> Vthev: %.2f V (Expected ~24.7V)", out.vthev[0]);
    printf("\nDrive: 0, Current: -1.0A -> Rthev: %.2f Ohm (Expected %.2f)\n", out.rthev[0], p.Rdson);

    return 0;
}