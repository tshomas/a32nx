/*
 * A32NX
 * Copyright (C) 2020 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SimConnectInterface.h"
#include <iostream>
#include <map>
#include <vector>

using namespace std;

bool SimConnectInterface::connect(bool autopilotStateMachineEnabled,
                                  bool autopilotLawsEnabled,
                                  bool flyByWireEnabled,
                                  const std::vector<std::shared_ptr<ThrottleAxisMapping>>& throttleAxis) {
  // info message
  cout << "WASM: Connecting..." << endl;

  // connect
  HRESULT result = SimConnect_Open(&hSimConnect, "FlyByWire", nullptr, 0, 0, 0);

  if (S_OK == result) {
    // we are now connected
    isConnected = true;
    cout << "WASM: Connected" << endl;
    // store throttle axis handler
    this->throttleAxis = throttleAxis;
    // add data to definition
    bool prepareResult = prepareSimDataSimConnectDataDefinitions();
    prepareResult &= prepareSimInputSimConnectDataDefinitions(autopilotStateMachineEnabled, autopilotLawsEnabled, flyByWireEnabled);
    prepareResult &= prepareSimOutputSimConnectDataDefinitions();
    prepareResult &= prepareClientDataDefinitions();
    // check result
    if (!prepareResult) {
      // failed to add data definition -> disconnect
      cout << "WASM: Failed to prepare data definitions" << endl;
      disconnect();
      // failed to connect
      return false;
    }
    // success
    return true;
  }
  // fallback -> failed
  return false;
}

void SimConnectInterface::disconnect() {
  if (isConnected) {
    // info message
    cout << "WASM: Disconnecting..." << endl;
    // close connection
    SimConnect_Close(hSimConnect);
    // set flag
    isConnected = false;
    // reset handle
    hSimConnect = 0;
    // info message
    cout << "WASM: Disconnected" << endl;
  }
}

bool SimConnectInterface::prepareSimDataSimConnectDataDefinitions() {
  bool result = true;

  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "G FORCE", "GFORCE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE PITCH DEGREES", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE BANK DEGREES", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_XYZ, "STRUCT BODY ROTATION VELOCITY", "STRUCT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_XYZ, "STRUCT BODY ROTATION ACCELERATION", "STRUCT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ACCELERATION BODY Z", "METER PER SECOND SQUARED");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ACCELERATION BODY X", "METER PER SECOND SQUARED");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ACCELERATION BODY Y", "METER PER SECOND SQUARED");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE HEADING DEGREES MAGNETIC", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE HEADING DEGREES TRUE", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GPS GROUND MAGNETIC TRACK", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ELEVATOR POSITION", "POSITION");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ELEVATOR TRIM POSITION", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AILERON POSITION", "POSITION");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "RUDDER POSITION", "POSITION");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "RUDDER TRIM PCT", "PERCENT OVER 100");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "INCIDENCE ALPHA", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "INCIDENCE BETA", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "BETA DOT", "DEGREE PER SECOND");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AIRSPEED INDICATED", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AIRSPEED TRUE", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AIRSPEED MACH", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GROUND VELOCITY", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PRESSURE ALTITUDE", "FEET");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "INDICATED ALTITUDE", "FEET");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE ALT ABOVE GROUND MINUS CG", "FEET");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "VELOCITY WORLD Y", "FEET PER MINUTE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "CG PERCENT", "PERCENT OVER 100");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TOTAL WEIGHT", "KILOGRAMS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GEAR ANIMATION POSITION:0", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GEAR ANIMATION POSITION:1", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GEAR ANIMATION POSITION:2", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "FLAPS HANDLE INDEX", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "SPOILERS LEFT POSITION", "PERCENT OVER 100");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "SPOILERS RIGHT POSITION", "PERCENT OVER 100");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "IS SLEW ACTIVE", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "AUTOPILOT MASTER", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "AUTOPILOT FLIGHT DIRECTOR ACTIVE:1", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "AUTOPILOT FLIGHT DIRECTOR ACTIVE:2", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AUTOPILOT AIRSPEED HOLD VAR", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AUTOPILOT ALTITUDE LOCK VAR:3", "FEET");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "SIMULATION TIME", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "SIMULATION RATE", "NUMBER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "STRUCTURAL ICE PCT", "PERCENT OVER 100");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "LINEAR CL ALPHA", "PER DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "STALL ALPHA", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "ZERO LIFT ALPHA", "DEGREE");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT DENSITY", "KILOGRAM PER CUBIC METER");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT PRESSURE", "MILLIBARS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT TEMPERATURE", "CELSIUS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT WIND X", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT WIND Y", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT WIND Z", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT WIND VELOCITY", "KNOTS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "AMBIENT WIND DIRECTION", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TOTAL AIR TEMPERATURE", "CELSIUS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE LATITUDE", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "PLANE LONGITUDE", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE LEVER POSITION:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE LEVER POSITION:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG JET THRUST:1", "POUNDS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG JET THRUST:2", "POUNDS");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "NAV HAS NAV:3", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "NAV LOCALIZER:3", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "NAV HAS DME:3", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "NAV DME:3", "NAUTICAL MILES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "NAV HAS LOCALIZER:3", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "NAV RADIAL ERROR:3", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "NAV HAS GLIDE SLOPE:3", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "NAV GLIDE SLOPE ERROR:3", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "AUTOTHROTTLE ACTIVE", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG CORRECTED N1:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG CORRECTED N1:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "GPS IS ACTIVE FLIGHT PLAN", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GPS WP CROSS TRK", "NAUTICAL MILES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "GPS WP TRACK ANGLE ERROR", "DEGREES");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG COMMANDED N1:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG COMMANDED N1:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG N1:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG N1:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG CORRECTED N1:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_FLOAT64, "TURB ENG CORRECTED N1:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "ENG COMBUSTION:1", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "ENG COMBUSTION:2", "BOOL");
  result &= addDataDefinition(hSimConnect, 0, SIMCONNECT_DATATYPE_INT64, "AUTOPILOT MANAGED SPEED IN MACH", "BOOL");

  return result;
}

bool SimConnectInterface::prepareSimInputSimConnectDataDefinitions(bool autopilotStateMachineEnabled,
                                                                   bool autopilotLawsEnabled,
                                                                   bool flyByWireEnabled) {
  bool result = true;

  result &= addInputDataDefinition(hSimConnect, 0, Events::AXIS_ELEVATOR_SET, "AXIS_ELEVATOR_SET", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::AXIS_AILERONS_SET, "AXIS_AILERONS_SET", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::AXIS_RUDDER_SET, "AXIS_RUDDER_SET", flyByWireEnabled);

  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_SET, "RUDDER_SET", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_LEFT, "RUDDER_LEFT", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_AXIS_PLUS, "RUDDER_AXIS_PLUS", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_CENTER, "RUDDER_CENTER", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_RIGHT, "RUDDER_RIGHT", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::RUDDER_AXIS_MINUS, "RUDDER_AXIS_MINUS", flyByWireEnabled);

  result &= addInputDataDefinition(hSimConnect, 0, Events::AILERON_SET, "AILERON_SET", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::AILERONS_LEFT, "AILERONS_LEFT", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::AILERONS_RIGHT, "AILERONS_RIGHT", flyByWireEnabled);

  result &= addInputDataDefinition(hSimConnect, 0, Events::CENTER_AILER_RUDDER, "CENTER_AILER_RUDDER", flyByWireEnabled);

  result &= addInputDataDefinition(hSimConnect, 0, Events::ELEVATOR_SET, "ELEVATOR_SET", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::ELEV_DOWN, "ELEV_DOWN", flyByWireEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::ELEV_UP, "ELEV_UP", flyByWireEnabled);

  result &= addInputDataDefinition(hSimConnect, 0, Events::AP_MASTER, "AP_MASTER", autopilotStateMachineEnabled);
  result &= addInputDataDefinition(hSimConnect, 0, Events::AUTOPILOT_OFF, "AUTOPILOT_OFF", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_AP_1_PUSH, "A32NX.FCU_AP_1_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_AP_2_PUSH, "A32NX.FCU_AP_2_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_HDG_PUSH, "A32NX.FCU_HDG_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_HDG_PULL, "A32NX.FCU_HDG_PULL", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_ALT_PUSH, "A32NX.FCU_ALT_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_ALT_PULL, "A32NX.FCU_ALT_PULL", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_VS_PUSH, "A32NX.FCU_VS_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_VS_PULL, "A32NX.FCU_VS_PULL", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_LOC_PUSH, "A32NX.FCU_LOC_PUSH", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_FCU_APPR_PUSH, "A32NX.FCU_APPR_PUSH", false);

  result &= addInputDataDefinition(hSimConnect, 0, Events::AUTO_THROTTLE_ARM, "AUTO_THROTTLE_ARM", true);

  result &=
      addInputDataDefinition(hSimConnect, 0, Events::A32NX_THROTTLE_MAPPING_LOAD_FROM_FILE, "A32NX.THROTTLE_MAPPING_LOAD_FROM_FILE", false);
  result &= addInputDataDefinition(hSimConnect, 0, Events::A32NX_THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES,
                                   "A32NX.THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES", false);
  result &=
      addInputDataDefinition(hSimConnect, 0, Events::A32NX_THROTTLE_MAPPING_SAVE_TO_FILE, "A32NX.THROTTLE_MAPPING_SAVE_TO_FILE", false);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_SET, "THROTTLE_SET", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_SET, "THROTTLE1_SET", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_SET, "THROTTLE2_SET", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_AXIS_SET_EX1, "THROTTLE_AXIS_SET_EX1", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_AXIS_SET_EX1, "THROTTLE1_AXIS_SET_EX1", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_AXIS_SET_EX1, "THROTTLE2_AXIS_SET_EX1", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_FULL, "THROTTLE_FULL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_CUT, "THROTTLE_CUT", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_INCR, "THROTTLE_INCR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_DECR, "THROTTLE_DECR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_INCR_SMALL, "THROTTLE_INCR_SMALL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_DECR_SMALL, "THROTTLE_DECR_SMALL", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_10, "THROTTLE_10", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_20, "THROTTLE_20", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_30, "THROTTLE_30", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_40, "THROTTLE_40", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_50, "THROTTLE_50", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_60, "THROTTLE_60", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_70, "THROTTLE_70", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_80, "THROTTLE_80", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_90, "THROTTLE_90", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_FULL, "THROTTLE1_FULL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_CUT, "THROTTLE1_CUT", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_INCR, "THROTTLE1_INCR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_DECR, "THROTTLE1_DECR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_INCR_SMALL, "THROTTLE1_INCR_SMALL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE1_DECR_SMALL, "THROTTLE1_DECR_SMALL", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_FULL, "THROTTLE2_FULL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_CUT, "THROTTLE2_CUT", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_INCR, "THROTTLE2_INCR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_DECR, "THROTTLE2_DECR", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_INCR_SMALL, "THROTTLE2_INCR_SMALL", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE2_DECR_SMALL, "THROTTLE2_DECR_SMALL", true);

  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_REVERSE_THRUST_TOGGLE, "THROTTLE_REVERSE_THRUST_TOGGLE", true);
  result &= addInputDataDefinition(hSimConnect, 0, Events::THROTTLE_REVERSE_THRUST_HOLD, "THROTTLE_REVERSE_THRUST_HOLD", true);

  return result;
}

bool SimConnectInterface::prepareSimOutputSimConnectDataDefinitions() {
  bool result = true;

  result &= addDataDefinition(hSimConnect, 1, SIMCONNECT_DATATYPE_FLOAT64, "ELEVATOR POSITION", "POSITION");
  result &= addDataDefinition(hSimConnect, 1, SIMCONNECT_DATATYPE_FLOAT64, "AILERON POSITION", "POSITION");
  result &= addDataDefinition(hSimConnect, 1, SIMCONNECT_DATATYPE_FLOAT64, "RUDDER POSITION", "POSITION");

  result &= addDataDefinition(hSimConnect, 2, SIMCONNECT_DATATYPE_FLOAT64, "ELEVATOR TRIM POSITION", "DEGREE");

  result &= addDataDefinition(hSimConnect, 3, SIMCONNECT_DATATYPE_FLOAT64, "RUDDER TRIM PCT", "PERCENT OVER 100");

  result &= addDataDefinition(hSimConnect, 4, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE LEVER POSITION:1", "PERCENT");
  result &= addDataDefinition(hSimConnect, 4, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE LEVER POSITION:2", "PERCENT");
  result &= addDataDefinition(hSimConnect, 4, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE MANAGED MODE:1", "NUMBER");
  result &= addDataDefinition(hSimConnect, 4, SIMCONNECT_DATATYPE_FLOAT64, "GENERAL ENG THROTTLE MANAGED MODE:2", "NUMBER");

  return result;
}

bool SimConnectInterface::prepareClientDataDefinitions() {
  // variable for result
  HRESULT result;

  // ------------------------------------------------------------------------------------------------------------------

  // map client id
  result = SimConnect_MapClientDataNameToID(hSimConnect, "A32NX_CLIENT_DATA_AUTOPILOT_STATE_MACHINE", ClientData::AUTOPILOT_STATE_MACHINE);
  // create client data
  result &= SimConnect_CreateClientData(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, sizeof(ClientDataAutopilotStateMachine),
                                        SIMCONNECT_CREATE_CLIENT_DATA_FLAG_DEFAULT);
  // add data definitions
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_INT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_INT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);

  // request data to be updated when set
  result &= SimConnect_RequestClientData(hSimConnect, ClientData::AUTOPILOT_STATE_MACHINE, ClientData::AUTOPILOT_STATE_MACHINE,
                                         ClientData::AUTOPILOT_STATE_MACHINE, SIMCONNECT_CLIENT_DATA_PERIOD_ON_SET);

  // ------------------------------------------------------------------------------------------------------------------

  // map client id
  result &= SimConnect_MapClientDataNameToID(hSimConnect, "A32NX_CLIENT_DATA_AUTOPILOT_LAWS", ClientData::AUTOPILOT_LAWS);
  // create client data
  result &= SimConnect_CreateClientData(hSimConnect, ClientData::AUTOPILOT_LAWS, sizeof(ClientDataAutopilotLaws),
                                        SIMCONNECT_CREATE_CLIENT_DATA_FLAG_DEFAULT);
  // add data definitions
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_INT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOPILOT_LAWS, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);

  // request data to be updated when set
  result &= SimConnect_RequestClientData(hSimConnect, ClientData::AUTOPILOT_LAWS, ClientData::AUTOPILOT_LAWS, ClientData::AUTOPILOT_LAWS,
                                         SIMCONNECT_CLIENT_DATA_PERIOD_ON_SET);

  // ------------------------------------------------------------------------------------------------------------------

  // map client id
  result &= SimConnect_MapClientDataNameToID(hSimConnect, "A32NX_CLIENT_DATA_AUTOTHRUST", ClientData::AUTOTHRUST);
  // create client data
  result &= SimConnect_CreateClientData(hSimConnect, ClientData::AUTOTHRUST, sizeof(ClientDataAutothrust),
                                        SIMCONNECT_CREATE_CLIENT_DATA_FLAG_DEFAULT);
  // add data definitions
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);

  // request data to be updated when set
  result &= SimConnect_RequestClientData(hSimConnect, ClientData::AUTOTHRUST, ClientData::AUTOTHRUST, ClientData::AUTOTHRUST,
                                         SIMCONNECT_CLIENT_DATA_PERIOD_ON_SET);

  // ------------------------------------------------------------------------------------------------------------------

  // map client id
  result &= SimConnect_MapClientDataNameToID(hSimConnect, "A32NX_CLIENT_DATA_LOCAL_VARIABLES", ClientData::LOCAL_VARIABLES);
  // create client data
  result &= SimConnect_CreateClientData(hSimConnect, ClientData::LOCAL_VARIABLES, sizeof(ClientDataLocalVariables),
                                        SIMCONNECT_CREATE_CLIENT_DATA_FLAG_DEFAULT);
  // add data definitions
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);

  // ------------------------------------------------------------------------------------------------------------------

  // map client id
  result &=
      SimConnect_MapClientDataNameToID(hSimConnect, "A32NX_CLIENT_DATA_LOCAL_VARIABLES_AUTOTHRUST", ClientData::LOCAL_VARIABLES_AUTOTHRUST);
  // create client
  result &= SimConnect_CreateClientData(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, sizeof(ClientDataLocalVariablesAutothrust),
                                        SIMCONNECT_CREATE_CLIENT_DATA_FLAG_DEFAULT);
  // add data definitions
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);
  result &= SimConnect_AddToClientDataDefinition(hSimConnect, ClientData::LOCAL_VARIABLES_AUTOTHRUST, SIMCONNECT_CLIENTDATAOFFSET_AUTO,
                                                 SIMCONNECT_CLIENTDATATYPE_FLOAT64);

  // ------------------------------------------------------------------------------------------------------------------

  // return result
  return SUCCEEDED(result);
}

bool SimConnectInterface::requestReadData() {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // request data
  if (!requestData()) {
    return false;
  }

  // read data
  if (!readData()) {
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::requestData() {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // request data
  HRESULT result = SimConnect_RequestDataOnSimObject(hSimConnect, 0, 0, SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE);

  // check result of data request
  if (result != S_OK) {
    // request failed
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::readData() {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // get next dispatch message(s) and process them
  DWORD cbData;
  SIMCONNECT_RECV* pData;
  while (SUCCEEDED(SimConnect_GetNextDispatch(hSimConnect, &pData, &cbData))) {
    simConnectProcessDispatchMessage(pData, &cbData);
  }

  // success
  return true;
}

bool SimConnectInterface::sendData(SimOutput output) {
  // write data and return result
  return sendData(1, sizeof(output), &output);
}

bool SimConnectInterface::sendData(SimOutputEtaTrim output) {
  // write data and return result
  return sendData(2, sizeof(output), &output);
}

bool SimConnectInterface::sendData(SimOutputZetaTrim output) {
  // write data and return result
  return sendData(3, sizeof(output), &output);
}

bool SimConnectInterface::sendData(SimOutputThrottles output) {
  // write data and return result
  return sendData(4, sizeof(output), &output);
}

bool SimConnectInterface::sendEvent(Events eventId) {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // send event
  HRESULT result = SimConnect_TransmitClientEvent(hSimConnect, 0, eventId, 0, SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                  SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY);

  // check result of data request
  if (result != S_OK) {
    // request failed
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::setClientDataLocalVariables(ClientDataLocalVariables output) {
  // write data and return result
  return sendClientData(ClientData::LOCAL_VARIABLES, sizeof(output), &output);
}

bool SimConnectInterface::setClientDataLocalVariablesAutothrust(ClientDataLocalVariablesAutothrust output) {
  // write data and return result
  return sendClientData(ClientData::LOCAL_VARIABLES_AUTOTHRUST, sizeof(output), &output);
}

SimData SimConnectInterface::getSimData() {
  return simData;
}

SimInput SimConnectInterface::getSimInput() {
  return simInput;
}

SimInputAutopilot SimConnectInterface::getSimInputAutopilot() {
  return simInputAutopilot;
}

SimInputThrottles SimConnectInterface::getSimInputThrottles() {
  return simInputThrottles;
}

void SimConnectInterface::resetSimInputAutopilot() {
  simInputAutopilot.AP_1_push = 0;
  simInputAutopilot.AP_2_push = 0;
  simInputAutopilot.AP_disconnect = 0;
  simInputAutopilot.HDG_push = 0;
  simInputAutopilot.HDG_pull = 0;
  simInputAutopilot.ALT_push = 0;
  simInputAutopilot.ALT_pull = 0;
  simInputAutopilot.VS_push = 0;
  simInputAutopilot.VS_pull = 0;
  simInputAutopilot.LOC_push = 0;
  simInputAutopilot.APPR_push = 0;
}

void SimConnectInterface::resetSimInputThrottles() {
  simInputThrottles.ATHR_push = 0;
}

bool SimConnectInterface::setClientDataAutopilotLaws(ClientDataAutopilotLaws output) {
  // write data and return result
  return sendClientData(ClientData::AUTOPILOT_LAWS, sizeof(output), &output);
}

ClientDataAutopilotLaws SimConnectInterface::getClientDataAutopilotLaws() {
  return clientDataAutopilotLaws;
}

bool SimConnectInterface::setClientDataAutopilotStateMachine(ClientDataAutopilotStateMachine output) {
  // write data and return result
  return sendClientData(ClientData::AUTOPILOT_STATE_MACHINE, sizeof(output), &output);
}

ClientDataAutopilotStateMachine SimConnectInterface::getClientDataAutopilotStateMachine() {
  return clientDataAutopilotStateMachine;
}

ClientDataAutothrust SimConnectInterface::getClientDataAutothrust() {
  return clientDataAutothrust;
}

void SimConnectInterface::simConnectProcessDispatchMessage(SIMCONNECT_RECV* pData, DWORD* cbData) {
  switch (pData->dwID) {
    case SIMCONNECT_RECV_ID_OPEN:
      // connection established
      cout << "WASM: SimConnect connection established" << endl;
      break;

    case SIMCONNECT_RECV_ID_QUIT:
      // connection lost
      cout << "WASM: Received SimConnect connection quit message" << endl;
      disconnect();
      break;

    case SIMCONNECT_RECV_ID_EVENT:
      // get event
      simConnectProcessEvent(static_cast<SIMCONNECT_RECV_EVENT*>(pData));
      break;

    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
      // process data
      simConnectProcessSimObjectData(static_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(pData));
      break;

    case SIMCONNECT_RECV_ID_CLIENT_DATA:
      // process data
      simConnectProcessClientData(static_cast<SIMCONNECT_RECV_CLIENT_DATA*>(pData));
      break;

    case SIMCONNECT_RECV_ID_EXCEPTION:
      // exception
      cout << "WASM: Exception in SimConnect connection: ";
      cout << getSimConnectExceptionString(static_cast<SIMCONNECT_EXCEPTION>(static_cast<SIMCONNECT_RECV_EXCEPTION*>(pData)->dwException));
      cout << endl;
      break;

    default:
      break;
  }
}

void SimConnectInterface::simConnectProcessEvent(const SIMCONNECT_RECV_EVENT* event) {
  // process depending on event id
  switch (event->uEventID) {
    case Events::AXIS_ELEVATOR_SET:
      simInput.inputs[AXIS_ELEVATOR_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::AXIS_AILERONS_SET:
      simInput.inputs[AXIS_AILERONS_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::AXIS_RUDDER_SET:
      simInput.inputs[AXIS_RUDDER_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::RUDDER_SET:
      simInput.inputs[AXIS_RUDDER_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::RUDDER_LEFT:
      simInput.inputs[AXIS_RUDDER_SET] = min(1.0, simInput.inputs[AXIS_RUDDER_SET] + 0.02);
      break;

    case Events::RUDDER_CENTER:
      simInput.inputs[AXIS_RUDDER_SET] = 0.0;
      break;

    case Events::RUDDER_RIGHT:
      simInput.inputs[AXIS_RUDDER_SET] = max(-1.0, simInput.inputs[AXIS_RUDDER_SET] - 0.02);
      break;

    case Events::RUDDER_AXIS_MINUS:
      simInput.inputs[AXIS_RUDDER_SET] = +1.0 * (static_cast<long>(event->dwData) / 16384.0);
      break;

    case Events::RUDDER_AXIS_PLUS:
      simInput.inputs[AXIS_RUDDER_SET] = -1.0 * (static_cast<long>(event->dwData) / 16384.0);
      break;

    case Events::AILERON_SET:
      simInput.inputs[AXIS_AILERONS_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::AILERONS_LEFT:
      simInput.inputs[AXIS_AILERONS_SET] = min(1.0, simInput.inputs[AXIS_AILERONS_SET] + 0.02);
      break;

    case Events::AILERONS_RIGHT:
      simInput.inputs[AXIS_AILERONS_SET] = max(-1.0, simInput.inputs[AXIS_AILERONS_SET] - 0.02);
      break;

    case Events::CENTER_AILER_RUDDER:
      simInput.inputs[AXIS_RUDDER_SET] = 0.0;
      simInput.inputs[AXIS_AILERONS_SET] = 0.0;
      break;

    case Events::ELEVATOR_SET:
      simInput.inputs[AXIS_ELEVATOR_SET] = static_cast<long>(event->dwData) / 16384.0;
      break;

    case Events::ELEV_DOWN:
      simInput.inputs[AXIS_ELEVATOR_SET] = min(1.0, simInput.inputs[AXIS_ELEVATOR_SET] + 0.02);
      break;

    case Events::ELEV_UP:
      simInput.inputs[AXIS_ELEVATOR_SET] = max(-1.0, simInput.inputs[AXIS_ELEVATOR_SET] - 0.02);
      break;

    case Events::AUTOPILOT_OFF: {
      simInputAutopilot.AP_disconnect = 1;
      cout << "WASM: event triggered: AUTOPILOT_OFF" << endl;
      break;
    }

    case Events::AP_MASTER: {
      simInputAutopilot.AP_1_push = 1;
      cout << "WASM: event triggered: AP_MASTER" << endl;
      break;
    }

    case Events::A32NX_FCU_AP_1_PUSH: {
      simInputAutopilot.AP_1_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_AP_1_PUSH" << endl;
      break;
    }

    case Events::A32NX_FCU_AP_2_PUSH: {
      simInputAutopilot.AP_2_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_AP_2_PUSH" << endl;
      break;
    }

    case Events::A32NX_FCU_HDG_PUSH: {
      simInputAutopilot.HDG_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_HDG_PUSH" << endl;
      break;
    }
    case Events::A32NX_FCU_HDG_PULL: {
      simInputAutopilot.HDG_pull = 1;
      cout << "WASM: event triggered: A32NX_FCU_HDG_PULL" << endl;
      break;
    }

    case Events::A32NX_FCU_ALT_PUSH: {
      simInputAutopilot.ALT_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_ALT_PUSH" << endl;
      break;
    }
    case Events::A32NX_FCU_ALT_PULL: {
      simInputAutopilot.ALT_pull = 1;
      cout << "WASM: event triggered: A32NX_FCU_ALT_PULL" << endl;
      break;
    }

    case Events::A32NX_FCU_VS_PUSH: {
      simInputAutopilot.VS_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_VS_PUSH" << endl;
      break;
    }
    case Events::A32NX_FCU_VS_PULL: {
      simInputAutopilot.VS_pull = 1;
      cout << "WASM: event triggered: A32NX_FCU_VS_PULL" << endl;
      break;
    }

    case Events::A32NX_FCU_LOC_PUSH: {
      simInputAutopilot.LOC_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_LOC_PUSH" << endl;
      break;
    }

    case Events::A32NX_FCU_APPR_PUSH: {
      simInputAutopilot.APPR_push = 1;
      cout << "WASM: event triggered: A32NX_FCU_APPR_PUSH" << endl;
      break;
    }

    case Events::AUTO_THROTTLE_ARM: {
      simInputThrottles.ATHR_push = 1;
      cout << "WASM: event triggered: AUTO_THROTTLE_ARM" << endl;
      break;
    }

    case Events::A32NX_THROTTLE_MAPPING_LOAD_FROM_FILE: {
      cout << "WASM: event triggered: THROTTLE_MAPPING_LOAD_FROM_FILE" << endl;
      throttleAxis[0]->loadFromFile();
      throttleAxis[1]->loadFromFile();
      break;
    }

    case Events::A32NX_THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES: {
      cout << "WASM: event triggered: THROTTLE_MAPPING_LOAD_FROM_LOCAL_VARIABLES" << endl;
      throttleAxis[0]->loadFromLocalVariables();
      throttleAxis[1]->loadFromLocalVariables();
      break;
    }

    case Events::A32NX_THROTTLE_MAPPING_SAVE_TO_FILE: {
      cout << "WASM: event triggered: THROTTLE_MAPPING_SAVE_TO_FILE" << endl;
      throttleAxis[0]->saveToFile();
      throttleAxis[1]->saveToFile();
      break;
    }

    case Events::THROTTLE_SET: {
      throttleAxis[0]->onEventThrottleSet(static_cast<long>(event->dwData));
      throttleAxis[1]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE1_SET: {
      throttleAxis[0]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE2_SET: {
      throttleAxis[1]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE_AXIS_SET_EX1: {
      throttleAxis[0]->onEventThrottleSet(static_cast<long>(event->dwData));
      throttleAxis[1]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE1_AXIS_SET_EX1: {
      throttleAxis[0]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE2_AXIS_SET_EX1: {
      throttleAxis[1]->onEventThrottleSet(static_cast<long>(event->dwData));
      break;
    }

    case Events::THROTTLE_FULL: {
      throttleAxis[0]->onEventThrottleFull();
      throttleAxis[1]->onEventThrottleFull();
      break;
    }

    case Events::THROTTLE_CUT: {
      throttleAxis[0]->onEventThrottleCut();
      throttleAxis[1]->onEventThrottleCut();
      break;
    }

    case Events::THROTTLE_INCR: {
      throttleAxis[0]->onEventThrottleIncrease();
      throttleAxis[1]->onEventThrottleIncrease();
      break;
    }

    case Events::THROTTLE_DECR: {
      throttleAxis[0]->onEventThrottleDecrease();
      throttleAxis[1]->onEventThrottleDecrease();
      break;
    }

    case Events::THROTTLE_INCR_SMALL: {
      throttleAxis[0]->onEventThrottleIncreaseSmall();
      throttleAxis[1]->onEventThrottleIncreaseSmall();
      break;
    }

    case Events::THROTTLE_DECR_SMALL: {
      throttleAxis[0]->onEventThrottleDecreaseSmall();
      throttleAxis[1]->onEventThrottleDecreaseSmall();
      break;
    }

    case Events::THROTTLE_10: {
      throttleAxis[0]->onEventThrottleSet_10();
      throttleAxis[1]->onEventThrottleSet_10();
      break;
    }

    case Events::THROTTLE_20:
      throttleAxis[0]->onEventThrottleSet_20();
      throttleAxis[1]->onEventThrottleSet_20();
      break;

    case Events::THROTTLE_30:
      throttleAxis[0]->onEventThrottleSet_30();
      throttleAxis[1]->onEventThrottleSet_30();
      break;

    case Events::THROTTLE_40:
      throttleAxis[0]->onEventThrottleSet_40();
      throttleAxis[1]->onEventThrottleSet_40();
      break;

    case Events::THROTTLE_50:
      throttleAxis[0]->onEventThrottleSet_50();
      throttleAxis[1]->onEventThrottleSet_50();
      break;

    case Events::THROTTLE_60:
      throttleAxis[0]->onEventThrottleSet_50();
      throttleAxis[1]->onEventThrottleSet_60();
      break;

    case Events::THROTTLE_70:
      throttleAxis[0]->onEventThrottleSet_70();
      throttleAxis[1]->onEventThrottleSet_70();
      break;

    case Events::THROTTLE_80:
      throttleAxis[0]->onEventThrottleSet_80();
      throttleAxis[1]->onEventThrottleSet_80();
      break;

    case Events::THROTTLE_90:
      throttleAxis[0]->onEventThrottleSet_90();
      throttleAxis[1]->onEventThrottleSet_90();
      break;

    case Events::THROTTLE1_FULL: {
      throttleAxis[0]->onEventThrottleFull();
      break;
    }

    case Events::THROTTLE1_CUT: {
      throttleAxis[0]->onEventThrottleCut();
      break;
    }

    case Events::THROTTLE1_INCR: {
      throttleAxis[0]->onEventThrottleIncrease();
      break;
    }

    case Events::THROTTLE1_DECR: {
      throttleAxis[0]->onEventThrottleDecrease();
      break;
    }

    case Events::THROTTLE1_INCR_SMALL: {
      throttleAxis[0]->onEventThrottleIncreaseSmall();
      break;
    }

    case Events::THROTTLE1_DECR_SMALL: {
      throttleAxis[0]->onEventThrottleDecreaseSmall();
      break;
    }

    case Events::THROTTLE2_FULL: {
      throttleAxis[1]->onEventThrottleFull();
      break;
    }

    case Events::THROTTLE2_CUT: {
      throttleAxis[1]->onEventThrottleCut();
      break;
    }

    case Events::THROTTLE2_INCR: {
      throttleAxis[1]->onEventThrottleIncrease();
      break;
    }

    case Events::THROTTLE2_DECR: {
      throttleAxis[1]->onEventThrottleDecrease();
      break;
    }

    case Events::THROTTLE2_INCR_SMALL: {
      throttleAxis[1]->onEventThrottleIncreaseSmall();
      break;
    }

    case Events::THROTTLE2_DECR_SMALL: {
      throttleAxis[1]->onEventThrottleDecreaseSmall();
      break;
    }

    case Events::THROTTLE_REVERSE_THRUST_TOGGLE: {
      throttleAxis[0]->onEventReverseToggle();
      throttleAxis[1]->onEventReverseToggle();
      break;
    }
    case Events::THROTTLE_REVERSE_THRUST_HOLD: {
      throttleAxis[0]->onEventReverseHold(static_cast<bool>(event->dwData));
      throttleAxis[1]->onEventReverseHold(static_cast<bool>(event->dwData));
      break;
    }

    default:
      break;
  }
}

void SimConnectInterface::simConnectProcessSimObjectData(const SIMCONNECT_RECV_SIMOBJECT_DATA* data) {
  // process depending on request id
  switch (data->dwRequestID) {
    case 0:
      // store aircraft data
      simData = *((SimData*)&data->dwData);
      return;

    default:
      // print unknown request id
      cout << "WASM: Unknown request id in SimConnect connection: ";
      cout << data->dwRequestID << endl;
      return;
  }
}

void SimConnectInterface::simConnectProcessClientData(const SIMCONNECT_RECV_CLIENT_DATA* data) {
  // process depending on request id
  switch (data->dwRequestID) {
    case ClientData::AUTOPILOT_STATE_MACHINE:
      // store aircraft data
      clientDataAutopilotStateMachine = *((ClientDataAutopilotStateMachine*)&data->dwData);
      return;

    case ClientData::AUTOPILOT_LAWS:
      // store aircraft data
      clientDataAutopilotLaws = *((ClientDataAutopilotLaws*)&data->dwData);
      return;

    case ClientData::AUTOTHRUST:
      // store aircraft data
      clientDataAutothrust = *((ClientDataAutothrust*)&data->dwData);
      return;

    default:
      // print unknown request id
      cout << "WASM: Unknown request id in SimConnect connection: ";
      cout << data->dwRequestID << endl;
      return;
  }
}

bool SimConnectInterface::sendClientData(SIMCONNECT_DATA_DEFINITION_ID id, DWORD size, void* data) {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // set output data
  HRESULT result = SimConnect_SetClientData(hSimConnect, id, id, SIMCONNECT_CLIENT_DATA_SET_FLAG_DEFAULT, 0, size, data);

  // check result of data request
  if (result != S_OK) {
    // request failed
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::sendData(SIMCONNECT_DATA_DEFINITION_ID id, DWORD size, void* data) {
  // check if we are connected
  if (!isConnected) {
    return false;
  }

  // set output data
  HRESULT result = SimConnect_SetDataOnSimObject(hSimConnect, id, SIMCONNECT_OBJECT_ID_USER, 0, 0, size, data);

  // check result of data request
  if (result != S_OK) {
    // request failed
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::addDataDefinition(const HANDLE connectionHandle,
                                            const SIMCONNECT_DATA_DEFINITION_ID id,
                                            const SIMCONNECT_DATATYPE dataType,
                                            const string& dataName,
                                            const string& dataUnit) {
  HRESULT result =
      SimConnect_AddToDataDefinition(connectionHandle, id, dataName.c_str(),
                                     SimConnectInterface::isSimConnectDataTypeStruct(dataType) ? nullptr : dataUnit.c_str(), dataType);

  return (result == S_OK);
}

bool SimConnectInterface::addInputDataDefinition(const HANDLE connectionHandle,
                                                 const SIMCONNECT_DATA_DEFINITION_ID groupId,
                                                 const SIMCONNECT_CLIENT_EVENT_ID eventId,
                                                 const string& eventName,
                                                 const bool maskEvent) {
  HRESULT result = SimConnect_MapClientEventToSimEvent(connectionHandle, eventId, eventName.c_str());

  if (result != S_OK) {
    // failed -> abort
    return false;
  }

  result = SimConnect_AddClientEventToNotificationGroup(connectionHandle, groupId, eventId, maskEvent ? TRUE : FALSE);
  if (result != S_OK) {
    // failed -> abort
    return false;
  }

  result = SimConnect_SetNotificationGroupPriority(connectionHandle, groupId, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);

  if (result != S_OK) {
    // failed -> abort
    return false;
  }

  // success
  return true;
}

bool SimConnectInterface::isSimConnectDataTypeStruct(SIMCONNECT_DATATYPE type) {
  switch (type) {
    case SIMCONNECT_DATATYPE_INITPOSITION:
    case SIMCONNECT_DATATYPE_MARKERSTATE:
    case SIMCONNECT_DATATYPE_WAYPOINT:
    case SIMCONNECT_DATATYPE_LATLONALT:
    case SIMCONNECT_DATATYPE_XYZ:
      return true;

    default:
      return false;
  }
  return false;
}

std::string SimConnectInterface::getSimConnectExceptionString(SIMCONNECT_EXCEPTION exception) {
  switch (exception) {
    case SIMCONNECT_EXCEPTION_NONE:
      return "NONE";

    case SIMCONNECT_EXCEPTION_ERROR:
      return "ERROR";

    case SIMCONNECT_EXCEPTION_SIZE_MISMATCH:
      return "SIZE_MISMATCH";

    case SIMCONNECT_EXCEPTION_UNRECOGNIZED_ID:
      return "UNRECOGNIZED_ID";

    case SIMCONNECT_EXCEPTION_UNOPENED:
      return "UNOPENED";

    case SIMCONNECT_EXCEPTION_VERSION_MISMATCH:
      return "VERSION_MISMATCH";

    case SIMCONNECT_EXCEPTION_TOO_MANY_GROUPS:
      return "TOO_MANY_GROUPS";

    case SIMCONNECT_EXCEPTION_NAME_UNRECOGNIZED:
      return "NAME_UNRECOGNIZED";

    case SIMCONNECT_EXCEPTION_TOO_MANY_EVENT_NAMES:
      return "TOO_MANY_EVENT_NAMES";

    case SIMCONNECT_EXCEPTION_EVENT_ID_DUPLICATE:
      return "EVENT_ID_DUPLICATE";

    case SIMCONNECT_EXCEPTION_TOO_MANY_MAPS:
      return "TOO_MANY_MAPS";

    case SIMCONNECT_EXCEPTION_TOO_MANY_OBJECTS:
      return "TOO_MANY_OBJECTS";

    case SIMCONNECT_EXCEPTION_TOO_MANY_REQUESTS:
      return "TOO_MANY_REQUESTS";

    case SIMCONNECT_EXCEPTION_WEATHER_INVALID_PORT:
      return "WEATHER_INVALID_PORT";

    case SIMCONNECT_EXCEPTION_WEATHER_INVALID_METAR:
      return "WEATHER_INVALID_METAR";

    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_GET_OBSERVATION:
      return "WEATHER_UNABLE_TO_GET_OBSERVATION";

    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_CREATE_STATION:
      return "WEATHER_UNABLE_TO_CREATE_STATION";

    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_REMOVE_STATION:
      return "WEATHER_UNABLE_TO_REMOVE_STATION";

    case SIMCONNECT_EXCEPTION_INVALID_DATA_TYPE:
      return "INVALID_DATA_TYPE";

    case SIMCONNECT_EXCEPTION_INVALID_DATA_SIZE:
      return "INVALID_DATA_SIZE";

    case SIMCONNECT_EXCEPTION_DATA_ERROR:
      return "DATA_ERROR";

    case SIMCONNECT_EXCEPTION_INVALID_ARRAY:
      return "INVALID_ARRAY";

    case SIMCONNECT_EXCEPTION_CREATE_OBJECT_FAILED:
      return "CREATE_OBJECT_FAILED";

    case SIMCONNECT_EXCEPTION_LOAD_FLIGHTPLAN_FAILED:
      return "LOAD_FLIGHTPLAN_FAILED";

    case SIMCONNECT_EXCEPTION_OPERATION_INVALID_FOR_OBJECT_TYPE:
      return "OPERATION_INVALID_FOR_OBJECT_TYPE";

    case SIMCONNECT_EXCEPTION_ILLEGAL_OPERATION:
      return "ILLEGAL_OPERATION";

    case SIMCONNECT_EXCEPTION_ALREADY_SUBSCRIBED:
      return "ALREADY_SUBSCRIBED";

    case SIMCONNECT_EXCEPTION_INVALID_ENUM:
      return "INVALID_ENUM";

    case SIMCONNECT_EXCEPTION_DEFINITION_ERROR:
      return "DEFINITION_ERROR";

    case SIMCONNECT_EXCEPTION_DUPLICATE_ID:
      return "DUPLICATE_ID";

    case SIMCONNECT_EXCEPTION_DATUM_ID:
      return "DATUM_ID";

    case SIMCONNECT_EXCEPTION_OUT_OF_BOUNDS:
      return "OUT_OF_BOUNDS";

    case SIMCONNECT_EXCEPTION_ALREADY_CREATED:
      return "ALREADY_CREATED";

    case SIMCONNECT_EXCEPTION_OBJECT_OUTSIDE_REALITY_BUBBLE:
      return "OBJECT_OUTSIDE_REALITY_BUBBLE";

    case SIMCONNECT_EXCEPTION_OBJECT_CONTAINER:
      return "OBJECT_CONTAINER";

    case SIMCONNECT_EXCEPTION_OBJECT_AI:
      return "OBJECT_AI";

    case SIMCONNECT_EXCEPTION_OBJECT_ATC:
      return "OBJECT_ATC";

    case SIMCONNECT_EXCEPTION_OBJECT_SCHEDULE:
      return "OBJECT_SCHEDULE";

    default:
      return "UNKNOWN";
  }
}
