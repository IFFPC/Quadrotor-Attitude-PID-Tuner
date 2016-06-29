package com.pappeln.dronepidtuner;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.provider.MediaStore;
import android.support.v7.app.AppCompatActivity;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.NumberPicker;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import android.widget.ProgressBar;
import android.app.ProgressDialog;

import com.o3dr.android.client.ControlTower;
import com.o3dr.android.client.Drone;

import com.o3dr.android.client.interfaces.DroneListener;
import com.o3dr.android.client.interfaces.TowerListener;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.connection.ConnectionParameter;
import com.o3dr.services.android.lib.drone.connection.ConnectionResult;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.Altitude;
import com.o3dr.services.android.lib.drone.property.Attitude;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.Home;
import com.o3dr.services.android.lib.drone.property.Speed;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;
import com.o3dr.services.android.lib.drone.property.VehicleMode;
import com.o3dr.services.android.lib.drone.property.Parameters;
import com.o3dr.services.android.lib.drone.property.Parameter;



import java.util.List;
import java.util.ArrayList;


/* NOTE: this program is written with  compile 'com.o3dr:3dr-services-lib:2.2.16', which is
 * a pretty old version of 3DR Services.
 * Refreshing parameters works pretty well on this version of lib, but is very unstable on
 * the most up-to-date version.
 * Some of the functions in this app maynot be compatable with
 * the most up-to-date library. This version of the service lib corresponds to
 * 3DR_Services V1.2.8 and Tower V3.1.3 apps.
 */

public class MainActivity extends AppCompatActivity implements DroneListener, TowerListener {

    private static final String TAG = MainActivity.class.getSimpleName();

    private Drone drone;
    private Parameters params;
    private int droneType = Type.TYPE_COPTER;
    private ControlTower controlTower;

    private final Handler handler = new Handler();
    private static final int DEFAULT_UDP_PORT = 14550;
    private static final int DEFAULT_USB_BAUD_RATE = 57600;

    private Spinner modeSelector;

    private double pitchRateP0 = 0;
    private double pitchRateI0 = 0;
    private double pitchRateD0 = 0;
    private double pitchRateFF0 = 0;
    private double pitchAngleP0 = 0;

    private double pitchRateP = 0;
    private double pitchRateI = 0;
    private double pitchRateD = 0;
    private double pitchRateFF = 0;
    private double pitchAngleP = 0;

    private double pitchRatePMax = 0;
    private double pitchRateIMax = 0;
    private double pitchRateDMax = 0;
    private double pitchRateFFMax = 0;
    private double pitchAnglePMax = 0;

    private double rollRateP0 = 0;
    private double rollRateI0 = 0;
    private double rollRateD0 = 0;
    private double rollRateFF0 = 0;
    private double rollAngleP0 = 0;

    private double rollRateP = 0;
    private double rollRateI = 0;
    private double rollRateD = 0;
    private double rollRateFF = 0;
    private double rollAngleP = 0;

    private double rollRatePMax = 0;
    private double rollRateIMax = 0;
    private double rollRateDMax = 0;
    private double rollRateFFMax = 0;
    private double rollAnglePMax = 0;


    private double yawRateP0 = 0;
    private double yawRateI0 = 0;
    private double yawRateD0 = 0;
    private double yawRateFF0 = 0;
    private double yawAngleP0 = 0;
    private double yawAngleFF0 = 0;

    private double yawRateP = 0;
    private double yawRateI = 0;
    private double yawRateD = 0;
    private double yawRateFF = 0;
    private double yawAngleP = 0;
    private double yawAngleFF = 0;

    private double yawRatePMax = 0;
    private double yawRateIMax = 0;
    private double yawRateDMax = 0;
    private double yawRateFFMax = 0;
    private double yawAnglePMax = 0;
    private double yawAngleFFMax = 0;

    private final int npMaxValue = 100;
    private final int npMinValue = 0;


    private ProgressDialog progressDialog;

    private ProgressBar mLoadingProgress;

    private boolean is_initial_refresh = true;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        final Context context = getApplicationContext();

        controlTower = new ControlTower(context);
        drone = new Drone();
        params = new Parameters();

        is_initial_refresh = true;

        mLoadingProgress = (ProgressBar) this.findViewById(R.id.reload_progress);

        mLoadingProgress.setVisibility(View.VISIBLE);
        mLoadingProgress.setIndeterminate(false);
        mLoadingProgress.setMax(100);
        mLoadingProgress.setProgress(0);


        modeSelector = (Spinner) findViewById(R.id.modeSelect);
        modeSelector.setOnItemSelectedListener(new Spinner.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                //onFlightModeSelected(view);
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // Do nothing
            }
        });
        Spinner connectionSelector = (Spinner) findViewById(R.id.selectConnectionType);
        connectionSelector.setSelection(1);

        // initialisation of the PID values
        initPIDValues();
        // initialisation of the RadioGroup
        initRadioGroup();
        // initialisation of the NumberPickers
        initNumberPickers();
        // initialisation of the EditTexts
        setEditTextsFromPIDValues();
        // update the UI display
        updateDisplayFromPIDValues();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onStart() {
        super.onStart();
        controlTower.connect(this);
        updateVehicleModesForType(droneType);
    }

    @Override
    public void onStop() {
        super.onStop();
        if (drone.isConnected()) {
            drone.disconnect();
            updateConnectedButton(false);
        }
        ;
        controlTower.unregisterDrone(drone);
        controlTower.disconnect();
    }

    // 3DR Services Listener
    // ==========================================================

    @Override
    public void onTowerConnected() {
        alertUser("3DR Services Connected");
        drone.unregisterDroneListener(this);
        controlTower.registerDrone(drone, handler);
        drone.registerDroneListener(this);
    }


    @Override
    public void onTowerDisconnected() {
        alertUser("3DR Service Interrupted");
    }

    // Drone Listener
    // ==========================================================

    @Override
    public void onDroneEvent(String event, Bundle extras) {

        switch (event) {
            case AttributeEvent.STATE_CONNECTED:
                alertUser("Drone Connected");
                updateConnectedButton(drone.isConnected());
                updateArmButton();
                // get parameters
                // A dialogue box will be displayed showing the progression
                drone.refreshParameters();

                break;

            case AttributeEvent.STATE_DISCONNECTED:
                alertUser("Drone Disconnected");
                updateConnectedButton(drone.isConnected());
                updateArmButton();
                break;

            case AttributeEvent.STATE_UPDATED:
            case AttributeEvent.STATE_ARMING:
                updateArmButton();
                break;

            case AttributeEvent.TYPE_UPDATED:
                Type newDroneType = drone.getAttribute(AttributeType.TYPE);
                if (newDroneType.getDroneType() != droneType) {
                    droneType = newDroneType.getDroneType();
                    updateVehicleModesForType(droneType);
                }
                break;

            case AttributeEvent.STATE_VEHICLE_MODE:
                updateVehicleMode();
                break;

            case AttributeEvent.SPEED_UPDATED:
                break;
/*
            case AttributeEvent.ALTITUDE_UPDATED:
                break;
*/
            case AttributeEvent.HOME_UPDATED:
                break;

            case AttributeEvent.PARAMETERS_RECEIVED:
                /* The following code does not work.
                final int defaultValue = -1;
                int index = extras.getInt(AttributeEventExtra.EXTRA_PARAMETER_INDEX, defaultValue);
                int count = extras.getInt(AttributeEventExtra.EXTRA_PARAMETERS_COUNT, defaultValue);
                     if (index != defaultValue && count != defaultValue)
                         updateProgress(index, count);
                */
                break;

            case AttributeEvent.PARAMETERS_REFRESH_STARTED:
                if (drone.isConnected()) {
                    startProgress();
                }
                break;

            case AttributeEvent.PARAMETERS_REFRESH_ENDED:

                //    alertUser("Parameters refreshed.");
                downloadPIDValuesFromDrone();
                // set display
                updateDisplayFromPIDValues();

                stopProgress();
                break;
            default:
//                Log.i("DRONE_EVENT", event); //Uncomment to see events from the drone
                break;
        }
    }

    @Override
    public void onDroneConnectionFailed(ConnectionResult result) {
        alertUser("Connection Failed:" + result.getErrorMessage());
    }

    @Override
    public void onDroneServiceInterrupted(String errorMsg) {

    }

    // UI Events
    // ==========================================================

    public void onBtnConnectTap(View view) {
        if (drone.isConnected()) {
            drone.disconnect();
        } else {
            Spinner connectionSelector = (Spinner) findViewById(R.id.selectConnectionType);
            int selectedConnectionType = connectionSelector.getSelectedItemPosition();

            Bundle extraParams = new Bundle();
            if (selectedConnectionType == ConnectionType.TYPE_USB) {
                extraParams.putInt(ConnectionType.EXTRA_USB_BAUD_RATE, DEFAULT_USB_BAUD_RATE); // Set default baud rate to 57600
            } else {
                extraParams.putInt(ConnectionType.EXTRA_UDP_SERVER_PORT, DEFAULT_UDP_PORT); // Set default baud rate to 14550
            }

            ConnectionParameter connectionParams = new ConnectionParameter(selectedConnectionType, extraParams, null);
            this.drone.connect(connectionParams);
        }

    }
/*
    public void onFlightModeSelected(View view) {
        VehicleMode vehicleMode = (VehicleMode) this.modeSelector.getSelectedItem();

        VehicleApi.getApi(this.drone).setVehicleMode(vehicleMode, new AbstractCommandListener() {
            @Override
            public void onSuccess() {
                alertUser("Vehicle mode change successful.");
            }

            @Override
            public void onError(int executionError) {
                alertUser("Vehicle mode change failed: " + executionError);
            }

            @Override
            public void onTimeout() {
                alertUser("Vehicle mode change timed out.");
            }
        });
    }
*/

    public void onArmButtonTap(View view) {
        State vehicleState = drone.getAttribute(AttributeType.STATE);
        if (vehicleState.isFlying()) {
            // Land
            drone.arm(false);
        } else if (vehicleState.isArmed()) {
            drone.arm(false);
        } else if (!vehicleState.isConnected()) {
            // Connect
            alertUser("Connect to a drone first");
        } else {
            // Connected but not Armed
            drone.arm(true);
        }
    }


    // UI updating
    // ==========================================================

    protected void updateConnectedButton(Boolean isConnected) {
        Button connectButton = (Button) findViewById(R.id.btnConnect);
        if (isConnected) {
            connectButton.setText("Disconnect");
        } else {
            connectButton.setText("Connect");
        }
    }

    protected void updateArmButton() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        Button armButton = (Button) findViewById(R.id.btnArmTakeOff);

        if (!this.drone.isConnected()) {
            armButton.setVisibility(View.INVISIBLE);
        } else {
            armButton.setVisibility(View.VISIBLE);
        }

        if (vehicleState.isFlying()) {
            // Land
            armButton.setText("LAND");
        } else if (vehicleState.isArmed()) {
            // Take off
            armButton.setText("TAKE OFF");
        } else if (vehicleState.isConnected()) {
            // Connected but not Armed
            armButton.setText("ARM");
        }
    }


    protected void updateVehicleModesForType(int droneType) {

        List<VehicleMode> vehicleModes = VehicleMode.getVehicleModePerDroneType(droneType);
        ArrayAdapter<VehicleMode> vehicleModeArrayAdapter = new ArrayAdapter<VehicleMode>(this, android.R.layout.simple_spinner_item, vehicleModes);
        vehicleModeArrayAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        this.modeSelector.setAdapter(vehicleModeArrayAdapter);
    }

    protected void updateVehicleMode() {
        State vehicleState = this.drone.getAttribute(AttributeType.STATE);
        VehicleMode vehicleMode = vehicleState.getVehicleMode();
        ArrayAdapter arrayAdapter = (ArrayAdapter) this.modeSelector.getAdapter();
        this.modeSelector.setSelection(arrayAdapter.getPosition(vehicleMode));
    }

    /*
     * when axis is changed, only display should be updated.
     */
    public void onRadioButtonClicked(View view){
        updateDisplayFromPIDValues();
        setEditTextsFromPIDValues();
    }
    public void onButtonResetClicked(View view){
        initPIDValues();
        setEditTextsFromPIDValues();
        updateDisplayFromPIDValues();
        uploadPIDValuesToDrone();

    }


    // Helper methods
    // ==========================================================

    protected void alertUser(String message) {
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_SHORT).show();
        Log.d(TAG, message);
    }

    protected double distanceBetweenPoints(LatLongAlt pointA, LatLongAlt pointB) {
        if (pointA == null || pointB == null) {
            return 0;
        }
        double dx = pointA.getLatitude() - pointB.getLatitude();
        double dy = pointA.getLongitude() - pointB.getLongitude();
        double dz = pointA.getAltitude() - pointB.getAltitude();
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private void startProgress() {

        progressDialog = new ProgressDialog(this);
        progressDialog.setTitle("Refreshing parameters...");
        progressDialog.setProgressStyle(ProgressDialog.STYLE_HORIZONTAL);
        progressDialog.setIndeterminate(true);
        progressDialog.setCancelable(false);
        progressDialog.setCanceledOnTouchOutside(true);
        progressDialog.show();

        mLoadingProgress.setIndeterminate(true);

        mLoadingProgress.setVisibility(View.VISIBLE);
    }

    private void updateProgress(int progress, int max) {
        if (progressDialog == null) {
            startProgress();
        }

        if (progressDialog.isIndeterminate()) {
            progressDialog.setIndeterminate(false);
            progressDialog.setMax(max);
        }
        progressDialog.setProgress(progress);

        if (mLoadingProgress.isIndeterminate()) {
            mLoadingProgress.setIndeterminate(false);
            mLoadingProgress.setMax(max);
        }
        mLoadingProgress.setProgress(progress);
    }

    private void stopProgress() {
        // dismiss progress dialog

        if (progressDialog != null) {
            progressDialog.dismiss();
            progressDialog = null;
        }
        mLoadingProgress.setIndeterminate(false);
        mLoadingProgress.setProgress(100);
        //mLoadingProgress.setVisibility(View.GONE);
    }
    /*
     * update the UI display from the PID values.
     */
    private void updateDisplayFromPIDValues(){
        RadioGroup rg;
        NumberPicker np;
        TextView tv;
        rg = (RadioGroup) findViewById(R.id.rgAxis);
        int axis = rg.getCheckedRadioButtonId();

        switch (axis) {
            case 1:  // pitch
                np = (NumberPicker) findViewById(R.id.npRateP);
                tv = (TextView) findViewById(R.id.tvRateP);
                tv.setText(String.format("%.3f",pitchRateP));
                np.setValue((int)Math.round(pitchRateP / pitchRatePMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateI);
                tv = (TextView) findViewById(R.id.tvRateI);
                tv.setText(String.format("%.3f",pitchRateI));
                np.setValue((int)Math.round(pitchRateI / pitchRateIMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateD);
                tv = (TextView) findViewById(R.id.tvRateD);
                tv.setText(String.format("%.3f",pitchRateD));
                np.setValue((int)Math.round(pitchRateD / pitchRateDMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateFF);
                tv = (TextView) findViewById(R.id.tvRateFF);
                tv.setText(String.format("%.3f",pitchRateFF));
                np.setValue((int)Math.round(pitchRateFF / pitchRateFFMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npAngleP);
                tv = (TextView) findViewById(R.id.tvAngleP);
                tv.setText(String.format("%.3f",pitchAngleP));
                np.setValue((int)Math.round(pitchAngleP / pitchAnglePMax * (double)(npMaxValue-npMinValue)));
                break;

            case 2: // roll
                np = (NumberPicker) findViewById(R.id.npRateP);
                tv = (TextView) findViewById(R.id.tvRateP);
                tv.setText(String.format("%.3f",rollRateP));
                np.setValue((int)Math.round(rollRateP / rollRatePMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateI);
                tv = (TextView) findViewById(R.id.tvRateI);
                tv.setText(String.format("%.3f",rollRateI));
                np.setValue((int)Math.round(rollRateI / rollRateIMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateD);
                tv = (TextView) findViewById(R.id.tvRateD);
                tv.setText(String.format("%.3f",rollRateD));
                np.setValue((int)Math.round(rollRateD / rollRateDMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateFF);
                tv = (TextView) findViewById(R.id.tvRateFF);
                tv.setText(String.format("%.3f",rollRateFF));
                np.setValue((int)Math.round(rollRateFF / rollRateFFMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npAngleP);
                tv = (TextView) findViewById(R.id.tvAngleP);
                tv.setText(String.format("%.3f",rollAngleP));
                np.setValue((int)Math.round(rollAngleP / rollAnglePMax * (double)(npMaxValue-npMinValue)));
                break;
            case 3: // yaw
                np = (NumberPicker) findViewById(R.id.npRateP);
                tv = (TextView) findViewById(R.id.tvRateP);
                tv.setText(String.format("%.3f",yawRateP));
                np.setValue((int)Math.round(yawRateP / yawRatePMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateI);
                tv = (TextView) findViewById(R.id.tvRateI);
                tv.setText(String.format("%.3f",yawRateI));
                np.setValue((int)Math.round(yawRateI / yawRateIMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateD);
                tv = (TextView) findViewById(R.id.tvRateD);
                tv.setText(String.format("%.3f",yawRateD));
                np.setValue((int)Math.round(yawRateD / yawRateDMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npRateFF);
                tv = (TextView) findViewById(R.id.tvRateFF);
                tv.setText(String.format("%.3f",yawRateFF));
                np.setValue((int)Math.round(yawRateFF / yawRateFFMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npAngleP);
                tv = (TextView) findViewById(R.id.tvAngleP);
                tv.setText(String.format("%.3f",yawAngleP));
                np.setValue((int)Math.round(yawAngleP / yawAnglePMax * (double)(npMaxValue-npMinValue)));

                np = (NumberPicker) findViewById(R.id.npAngleFF);
                tv = (TextView) findViewById(R.id.tvAngleFF);
                tv.setText(String.format("%.3f",yawAngleFF));
                np.setValue((int)Math.round(yawAngleFF / yawAngleFFMax * (double)(npMaxValue-npMinValue)));

                break;
            default:
                break;

        }

    }

    /*
     * update the PID values in this class, which serves as the data centre. i.e. source of any
     * access and sink of any update.
     */
    private void updatePIDValues() {
        RadioGroup rg = (RadioGroup) findViewById(R.id.rgAxis);
        int axis = rg.getCheckedRadioButtonId();
        NumberPicker np;
        EditText et;
        switch (axis) {
            case 1:  // pitch
                np = (NumberPicker) findViewById(R.id.npRateP);
                et = (EditText) findViewById(R.id.etRateP);
                pitchRatePMax = Double.parseDouble(et.getText().toString());
                pitchRateP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * pitchRatePMax;

                np = (NumberPicker) findViewById(R.id.npRateI);
                et = (EditText) findViewById(R.id.etRateI);
                pitchRateIMax = Double.parseDouble(et.getText().toString());
                pitchRateI = (double)np.getValue()/(double)(npMaxValue-npMinValue) * pitchRateIMax;

                np = (NumberPicker) findViewById(R.id.npRateD);
                et = (EditText) findViewById(R.id.etRateD);
                pitchRateDMax = Double.parseDouble(et.getText().toString());
                pitchRateD = (double)np.getValue()/(double)(npMaxValue-npMinValue) * pitchRateDMax;


                np = (NumberPicker) findViewById(R.id.npRateFF);
                et = (EditText) findViewById(R.id.etRateFF);
                pitchRateFFMax = Double.parseDouble(et.getText().toString());
                pitchRateFF = (double)np.getValue()/(double)(npMaxValue-npMinValue) * pitchRateFFMax;


                np = (NumberPicker) findViewById(R.id.npAngleP);
                et = (EditText) findViewById(R.id.etAngleP);
                pitchAnglePMax = Double.parseDouble(et.getText().toString());
                pitchAngleP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * pitchAnglePMax;


                break;
            case 2:  // roll
                np = (NumberPicker) findViewById(R.id.npRateP);
                et = (EditText) findViewById(R.id.etRateP);
                rollRatePMax = Double.parseDouble(et.getText().toString());
                rollRateP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * rollRatePMax;

                np = (NumberPicker) findViewById(R.id.npRateI);
                et = (EditText) findViewById(R.id.etRateI);
                rollRateIMax = Double.parseDouble(et.getText().toString());
                rollRateI = (double)np.getValue()/(double)(npMaxValue-npMinValue) * rollRateIMax;

                np = (NumberPicker) findViewById(R.id.npRateD);
                et = (EditText) findViewById(R.id.etRateD);
                rollRateDMax = Double.parseDouble(et.getText().toString());
                rollRateD = (double)np.getValue()/(double)(npMaxValue-npMinValue) * rollRateDMax;


                np = (NumberPicker) findViewById(R.id.npRateFF);
                et = (EditText) findViewById(R.id.etRateFF);
                rollRateFFMax = Double.parseDouble(et.getText().toString());
                rollRateFF = (double)np.getValue()/(double)(npMaxValue-npMinValue) * rollRateFFMax;


                np = (NumberPicker) findViewById(R.id.npAngleP);
                et = (EditText) findViewById(R.id.etAngleP);
                rollAnglePMax = Double.parseDouble(et.getText().toString());
                rollAngleP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * rollAnglePMax;
                break;

            case 3:  //yaw

                np = (NumberPicker) findViewById(R.id.npRateP);
                et = (EditText) findViewById(R.id.etRateP);
                yawRatePMax = Double.parseDouble(et.getText().toString());
                yawRateP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawRatePMax;

                np = (NumberPicker) findViewById(R.id.npRateI);
                et = (EditText) findViewById(R.id.etRateI);
                yawRateIMax = Double.parseDouble(et.getText().toString());
                yawRateI = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawRateIMax;

                np = (NumberPicker) findViewById(R.id.npRateD);
                et = (EditText) findViewById(R.id.etRateD);
                yawRateDMax = Double.parseDouble(et.getText().toString());
                yawRateD = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawRateDMax;


                np = (NumberPicker) findViewById(R.id.npRateFF);
                et = (EditText) findViewById(R.id.etRateFF);
                yawRateFFMax = Double.parseDouble(et.getText().toString());
                yawRateFF = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawRateFFMax;


                np = (NumberPicker) findViewById(R.id.npAngleP);
                et = (EditText) findViewById(R.id.etAngleP);
                yawAnglePMax = Double.parseDouble(et.getText().toString());
                yawAngleP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawAnglePMax;


                np = (NumberPicker) findViewById(R.id.npAngleP);
                et = (EditText) findViewById(R.id.etAngleP);
                yawAnglePMax = Double.parseDouble(et.getText().toString());
                yawAngleP = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawAnglePMax;


                np = (NumberPicker) findViewById(R.id.npAngleFF);
                et = (EditText) findViewById(R.id.etAngleFF);
                yawAngleFFMax = Double.parseDouble(et.getText().toString());
                yawAngleFF = (double)np.getValue()/(double)(npMaxValue-npMinValue) * yawAngleFFMax;

                break;
            default:
                break;

        }
    }

    public void initPIDValues(){
        // roll
        pitchRateP = pitchRateP0;
        pitchRateI = pitchRateI0;
        pitchRateD = pitchRateD0;
        pitchRateFF = pitchRateFF0;
        pitchAngleP = pitchAngleP0;
        // roll
        rollRateP = rollRateP0;
        rollRateI = rollRateI0;
        rollRateD = rollRateD0;
        rollRateFF = rollRateFF0;
        rollAngleP = rollAngleP0;
        // yaw
        yawRateP = yawRateP0;
        yawRateI = yawRateI0;
        yawRateD = yawRateD0;
        yawRateFF = yawRateFF0;
        yawAngleP = yawAngleP0;
        yawAngleFF = yawAngleFF0;

        // max values
        pitchRatePMax = 1;
        pitchRateIMax = 1;
        pitchRateDMax = 0.1;
        pitchRateFFMax = 0.01;
        pitchAnglePMax = 20;
        rollRatePMax = 1;
        rollRateIMax = 1;
        rollRateDMax = 0.1;
        rollRateFFMax = 0.01;
        rollAnglePMax = 20;
        yawRatePMax = 1;
        yawRateIMax = 1;
        yawRateDMax = 0.1;
        yawRateFFMax = 0.01;
        yawAnglePMax = 20;
        yawAngleFFMax = 0.01;
    }

    /*
     * set of the EditTexts, should be performed after the initialisation of PID values.
     */
    private void setEditTextsFromPIDValues(){
        RadioGroup rg = (RadioGroup) findViewById(R.id.rgAxis);
        int axis = rg.getCheckedRadioButtonId();
        EditText et;
        switch (axis) {
            case 1:  // pitch
                et = (EditText) findViewById(R.id.etRateP);
                et.setText(String.format("%.2f", pitchRatePMax));
                et = (EditText) findViewById(R.id.etRateI);
                et.setText(String.format("%.2f", pitchRateIMax));
                et = (EditText) findViewById(R.id.etRateD);
                et.setText(String.format("%.2f", pitchRateDMax));
                et = (EditText) findViewById(R.id.etRateFF);
                et.setText(String.format("%.2f", pitchRateFFMax));
                et = (EditText) findViewById(R.id.etAngleP);
                et.setText(String.format("%.2f", pitchAnglePMax));
                et = (EditText) findViewById(R.id.etAngleFF);
                et.setText(String.format("%.2f", 0.0));
                break;
            case 2: // roll
                et = (EditText) findViewById(R.id.etRateP);
                et.setText(String.format("%.2f", rollRatePMax));
                et = (EditText) findViewById(R.id.etRateI);
                et.setText(String.format("%.2f", rollRateIMax));
                et = (EditText) findViewById(R.id.etRateD);
                et.setText(String.format("%.2f", rollRateDMax));
                et = (EditText) findViewById(R.id.etRateFF);
                et.setText(String.format("%.2f", rollRateFFMax));
                et = (EditText) findViewById(R.id.etAngleP);
                et.setText(String.format("%.2f", rollAnglePMax));
                et = (EditText) findViewById(R.id.etAngleFF);
                et.setText(String.format("%.2f", 0.0));
                break;
            default: // yaw and default values
                et = (EditText) findViewById(R.id.etRateP);
                et.setText(String.format("%.2f", yawRatePMax));
                et = (EditText) findViewById(R.id.etRateI);
                et.setText(String.format("%.2f", yawRateIMax));
                et = (EditText) findViewById(R.id.etRateD);
                et.setText(String.format("%.2f", yawRateDMax));
                et = (EditText) findViewById(R.id.etRateFF);
                et.setText(String.format("%.2f", yawRateFFMax));
                et = (EditText) findViewById(R.id.etAngleP);
                et.setText(String.format("%.2f", yawAnglePMax));
                et = (EditText) findViewById(R.id.etAngleFF);
                et.setText(String.format("%.2f", yawAngleFFMax));
                break;
        }
    }


    /*
     * Initialisation of the RadioGroup
     */
    private void initRadioGroup(){
        RadioGroup rg = (RadioGroup) findViewById(R.id.rgAxis);
        ((RadioButton)rg.getChildAt(0)).setChecked(true);
    }

    /*
     * Initialisation of the number pickers.
     */
    private void initNumberPickers(){
        NumberPicker nb = (NumberPicker) findViewById(R.id.npRateP);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
        nb = (NumberPicker) findViewById(R.id.npRateI);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
        nb = (NumberPicker) findViewById(R.id.npRateD);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
        nb = (NumberPicker) findViewById(R.id.npRateFF);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
        nb = (NumberPicker) findViewById(R.id.npAngleP);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
        nb = (NumberPicker) findViewById(R.id.npAngleFF);
        nb.setMinValue(npMinValue);
        nb.setMaxValue(npMaxValue);
        nb.setWrapSelectorWheel(false);
        nb.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                updatePIDValues();
                updateDisplayFromPIDValues();
                uploadPIDValuesToDrone();
            }
        });
    }

    /*
     * write the PID parameters to Drone
     */
    public void uploadPIDValuesToDrone(){
        if(drone.isConnected()){
            mLoadingProgress.setIndeterminate(false);
            mLoadingProgress.setProgress(0);
            RadioGroup rg = (RadioGroup) findViewById(R.id.rgAxis);
            int axis = rg.getCheckedRadioButtonId();
            EditText et;
            Parameter paramTemp;
            List<Parameter> parametersList;
            switch (axis) {
                case 1:  // pitch
                    parametersList = new ArrayList<Parameter>(5);
                    paramTemp = params.getParameter("MC_PITCHRATE_P");
                    paramTemp.setValue(pitchRateP);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_PITCHRATE_I");
                    paramTemp.setValue(pitchRateI);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_PITCHRATE_D");
                    paramTemp.setValue(pitchRateD);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_PITCHRATE_FF");
                    paramTemp.setValue(pitchRateFF);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_PITCH_P");
                    paramTemp.setValue(pitchAngleP);
                    parametersList.add(paramTemp);
                    break;
                case 2:
                    parametersList = new ArrayList<Parameter>(5);
                    paramTemp = params.getParameter("MC_ROLLRATE_P");
                    paramTemp.setValue(rollRateP);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_ROLLRATE_I");
                    paramTemp.setValue(rollRateI);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_ROLLRATE_D");
                    paramTemp.setValue(rollRateD);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_ROLLRATE_FF");
                    paramTemp.setValue(rollRateFF);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_ROLL_P");
                    paramTemp.setValue(rollAngleP);
                    parametersList.add(paramTemp);
                    break;
                default: // yaw
                    parametersList = new ArrayList<Parameter>(6);
                    paramTemp = params.getParameter("MC_YAWRATE_P");
                    paramTemp.setValue(yawRateP);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_YAWRATE_I");
                    paramTemp.setValue(yawRateI);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_YAWRATE_D");
                    paramTemp.setValue(yawRateD);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_YAWRATE_FF");
                    paramTemp.setValue(yawRateFF);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_YAW_P");
                    paramTemp.setValue(yawAngleP);
                    parametersList.add(paramTemp);
                    paramTemp = params.getParameter("MC_YAW_FF");
                    paramTemp.setValue(yawAngleFF);
                    parametersList.add(paramTemp);
                    break;
            }
            drone.writeParameters(new Parameters(parametersList));
        }
    }
    /*
     * download the drone's parameter. Note: this function should be used after the
     * parameter refreshing is finished and the drone is connected. Otherwise program may crash.
     */
    public void downloadPIDValuesFromDrone(){
        params = drone.getAttribute(AttributeType.PARAMETERS);
        Parameter paramTemp;
        if (params != null) {
            if (is_initial_refresh) {
                //set initial values
                is_initial_refresh = false;
                // pitch
                paramTemp = params.getParameter("MC_PITCHRATE_P");
                pitchRateP0 = paramTemp.getValue();
                pitchRateP = pitchRateP0;
                paramTemp = params.getParameter("MC_PITCHRATE_I");
                pitchRateI0 = paramTemp.getValue();
                pitchRateI = pitchRateI0;
                paramTemp = params.getParameter("MC_PITCHRATE_D");
                pitchRateD0 = paramTemp.getValue();
                pitchRateD = pitchRateD0;
                paramTemp = params.getParameter("MC_PITCHRATE_FF");
                pitchRateFF0 = paramTemp.getValue();
                pitchRateFF = pitchRateFF0;
                paramTemp = params.getParameter("MC_PITCH_P");
                pitchAngleP0 = paramTemp.getValue();
                pitchAngleP = pitchAngleP0;
                // roll
                paramTemp = params.getParameter("MC_ROLLRATE_P");
                rollRateP0 = paramTemp.getValue();
                rollRateP = rollRateP0;
                paramTemp = params.getParameter("MC_ROLLRATE_I");
                rollRateI0 = paramTemp.getValue();
                rollRateI = rollRateI0;
                paramTemp = params.getParameter("MC_ROLLRATE_D");
                rollRateD0 = paramTemp.getValue();
                rollRateD = rollRateD0;
                paramTemp = params.getParameter("MC_ROLLRATE_FF");
                rollRateFF0 = paramTemp.getValue();
                rollRateFF = rollRateFF0;
                paramTemp = params.getParameter("MC_ROLL_P");
                rollAngleP0 = paramTemp.getValue();
                rollAngleP = rollAngleP0;
                // yaw
                paramTemp = params.getParameter("MC_YAWRATE_P");
                yawRateP0 = paramTemp.getValue();
                yawRateP = yawRateP0;
                paramTemp = params.getParameter("MC_YAWRATE_I");
                yawRateI0 = paramTemp.getValue();
                yawRateI = yawRateI0;
                paramTemp = params.getParameter("MC_YAWRATE_D");
                yawRateD0 = paramTemp.getValue();
                yawRateD = yawRateD0;
                paramTemp = params.getParameter("MC_YAWRATE_FF");
                yawRateFF0 = paramTemp.getValue();
                yawRateFF = yawRateFF0;
                paramTemp = params.getParameter("MC_YAW_P");
                yawAngleP0 = paramTemp.getValue();
                yawAngleP = yawAngleP0;
                paramTemp = params.getParameter("MC_YAW_FF");
                yawAngleFF0 = paramTemp.getValue();
                yawAngleFF = yawAngleFF0;
            } else {
                //get the actual PID values
                // pitch
                paramTemp = params.getParameter("MC_PITCHRATE_P");
                pitchRateP= paramTemp.getValue();
                paramTemp = params.getParameter("MC_PITCHRATE_I");
                pitchRateI= paramTemp.getValue();
                paramTemp = params.getParameter("MC_PITCHRATE_D");
                pitchRateD= paramTemp.getValue();
                paramTemp = params.getParameter("MC_PITCHRATE_FF");
                pitchRateFF= paramTemp.getValue();
                paramTemp = params.getParameter("MC_PITCH_P");
                pitchAngleP= paramTemp.getValue();
                // roll
                paramTemp = params.getParameter("MC_ROLLRATE_P");
                rollRateP= paramTemp.getValue();
                paramTemp = params.getParameter("MC_ROLLRATE_I");
                rollRateI= paramTemp.getValue();
                paramTemp = params.getParameter("MC_ROLLRATE_D");
                rollRateD= paramTemp.getValue();
                paramTemp = params.getParameter("MC_ROLLRATE_FF");
                rollRateFF= paramTemp.getValue();
                paramTemp = params.getParameter("MC_ROLL_P");
                rollAngleP= paramTemp.getValue();
                // yaw
                paramTemp = params.getParameter("MC_YAWRATE_P");
                yawRateP= paramTemp.getValue();
                paramTemp = params.getParameter("MC_YAWRATE_I");
                yawRateI= paramTemp.getValue();
                paramTemp = params.getParameter("MC_YAWRATE_D");
                yawRateD= paramTemp.getValue();
                paramTemp = params.getParameter("MC_YAWRATE_FF");
                yawRateFF= paramTemp.getValue();
                paramTemp = params.getParameter("MC_YAW_P");
                yawAngleP= paramTemp.getValue();
                paramTemp = params.getParameter("MC_YAW_FF");
                yawAngleFF= paramTemp.getValue();
            }
        }
    }
}