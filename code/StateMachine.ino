void switchStates(int switchFlightState)
{
    // Check for current state: GroundIdle, Countdown, Launch, Flight, Return, Landed, LaunchAbort, FlightAbort
    switch(switchFlightState)
    {
        case 1: // Ground Idle - sensor logging directly to SD, wating for launch signal
        GroundIdleState();
        break;

        case 2: // Countdown - Countdown, SD card logging suspended, using FLASH instead
        CountdownState();
        break;

        case 3: // Launch - first few seconds of the flight checking for flight nominality, if launch is not detected, switch to LaunchAbort
        LaunchState();
        break;

        case 4: // Flight - stabilization online, complete loging, chechks for flight nominality or return state conditions, if flight is not nominal, switch to FlightAbort
        FlightState();
        break;

        case 5: // Return - MECO, deploy chutes at safe atitude, wait for landing
        ReturnState();
        break;

        case 6: // Landed - landing detected, transfer data from flash to SD
        LandedState();
        break;

        case 7: // LaunchAbort - error at countdown -> stop countdown, transfer data from flash to SD, add error log to SD
        LaunchAbortState();
        break;

        case 8: // FlightAbort - jiggle engine vector to bleed as much power as possible, extended logging to flash, special chute conditions, wait for landing
        FlightAbortState();
        break;  
    }
}