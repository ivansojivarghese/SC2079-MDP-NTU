package com.example.mdp_group_35;

import static com.example.mdp_group_35.Home.refreshMessageReceivedNS;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import android.view.View;
import android.view.MotionEvent;
import android.os.Handler;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import androidx.appcompat.app.AppCompatActivity;
import java.util.Arrays;

import java.util.Arrays;

public class
ControlFragment extends Fragment implements SensorEventListener {
    private static final String TAG = "ControlFragment";

    SharedPreferences sharedPreferences;

    // Control Button
    ImageButton moveForwardImageBtn, turnRightImageBtn, moveBackImageBtn, turnLeftImageBtn,turnbleftImageBtn,turnbrightImageBtn;
    ImageButton exploreResetButton, fastestResetButton;
    private static long exploreTimer, fastestTimer;
    public static ToggleButton exploreButton, fastestButton;
    public static TextView exploreTimeTextView, fastestTimeTextView, robotStatusTextView;
    private static GridMap gridMap;


    private SensorManager sensorManager;
    private Sensor gyroSensor;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Initialize SensorManager
        sensorManager = (SensorManager) requireActivity().getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager == null) {
            Log.e("Gyro", "SensorManager NOT found!");
            return;
        }
        if (sensorManager != null) {
            gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        }

        if (gyroSensor == null) {
            Log.e("Gyro", "Gyroscope sensor NOT found on this device!");
        } else {
            Log.d("Gyro", "Gyroscope sensor successfully initialized");
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        if (gyroSensor != null) {
            sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_NORMAL);
            Log.d("Gyro", "Gyroscope Sensor Registered");
        } else {
            Log.e("Gyro", "Gyroscope Sensor NOT available");
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
        Log.d("Gyro", "Gyroscope Sensor Unregistered");
    }

    private float initialRotationY = Float.NaN;  // Store initial calibration value
    private static final float ALPHA = 0.2f; // Adjust this value to control smoothing
    private float smoothedRotationY = 0;

    public boolean isForwardButtonHeld = true;
    public boolean isBackwardButtonHeld = false;

    // Define a cooldown threshold (in milliseconds)
    private static final long TURN_COOLDOWN_MS = 1000;
    private long lastTurnTime = 0; // Store last turn timestamp

    // ✅ Mandatory method 1: Handle sensor changes
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (MappingFragment.gyroStatus) {
            if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                float rotationY = event.values[1]; // Y-axis rotation (tilt left/right)

                // Apply low-pass filter
                smoothedRotationY = ALPHA * rotationY + (1 - ALPHA) * smoothedRotationY;

                // Update baseline when there's minimal movement
                if (Math.abs(smoothedRotationY) < 0.5f) {
                    initialRotationY = smoothedRotationY;
                }

                // Compute relative rotation
                float relativeRotationY = smoothedRotationY - initialRotationY;

                // Get current time in milliseconds
                long currentTime = System.currentTimeMillis();

                if (relativeRotationY > 0.5f && (currentTime - lastTurnTime) > TURN_COOLDOWN_MS) {
                    Log.d("Gyro", "Tilting RIGHT 🚀");
                    lastTurnTime = currentTime; // Update cooldown timer

                    if (isForwardButtonHeld) {
                        showLog("Clicked turnRightImageBtn");
                        if (gridMap.getCanDrawRobot()) {
                            gridMap.moveRobot("right");
                            Home.refreshLabel();
                            Home.printMessage(Target.STM, "FC090");
                        } else {
                            updateStatus("Please press 'SET START POINT'");
                        }
                        showLog("Exiting turnRightImageBtn");
                    } else if (isBackwardButtonHeld) {
                        showLog("Clicked turnbRightImageBtn");
                        if (gridMap.getCanDrawRobot()) {
                            gridMap.moveRobot("backright");
                            Home.refreshLabel();
                            Home.printMessage(Target.STM, "BC090");
                        } else {
                            updateStatus("Please press 'SET START POINT'");
                        }
                        showLog("Exiting turnbRightImageBtn");
                    }

                } else if (relativeRotationY < -0.5f && (currentTime - lastTurnTime) > TURN_COOLDOWN_MS) {
                    Log.d("Gyro", "Tilting LEFT 🔄");
                    lastTurnTime = currentTime; // Update cooldown timer

                    if (isForwardButtonHeld) {
                        showLog("Clicked turnLeftImageBtn");
                        if (gridMap.getCanDrawRobot()) {
                            gridMap.moveRobot("left");
                            Home.refreshLabel();
                            Home.printMessage(Target.STM, "FA090");
                        } else {
                            updateStatus("Please press 'SET START POINT'");
                        }
                        showLog("Exiting turnLeftImageBtn");
                    } else if (isBackwardButtonHeld) {
                        showLog("Clicked turnbLeftImageBtn");
                        if (gridMap.getCanDrawRobot()) {
                            gridMap.moveRobot("backleft");
                            Home.refreshLabel();
                            Home.printMessage(Target.STM, "BA090");
                        } else {
                            updateStatus("Please press 'SET START POINT'");
                        }
                        showLog("Exiting turnbLeftImageBtn");
                    }
                } else {
                    Log.d("Gyro", "Stable position (ignoring small movements)");
                }
            }
        }
    }



    // ✅ Mandatory method 2: Handle accuracy changes (even if unused)
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // You can leave this empty or add logic if needed
    }

    // Timer
    public static Handler timerHandler = new Handler();

    //Button startSend;

    public static Runnable timerRunnableExplore = new Runnable() {
        @Override
        public void run() {
            long millisExplore = System.currentTimeMillis() - exploreTimer;
            int secondsExplore = (int) (millisExplore / 1000);
            int minutesExplore = secondsExplore / 60;
            secondsExplore = secondsExplore % 60;

            if (!Home.stopTimerFlag) {
                exploreTimeTextView.setText(String.format("%02d:%02d", minutesExplore,
                        secondsExplore));
                timerHandler.postDelayed(this, 500);
            }
        }
    };

    public static Runnable timerRunnableFastest = new Runnable() {
        @Override
        public void run() {
            long millisFastest = System.currentTimeMillis() - fastestTimer;
            int secondsFastest = (int) (millisFastest / 1000);
            int minutesFastest = secondsFastest / 60;
            secondsFastest = secondsFastest % 60;

            if (!Home.stopWk9TimerFlag) {
                fastestTimeTextView.setText(String.format("%02d:%02d", minutesFastest,
                        secondsFastest));
                timerHandler.postDelayed(this, 500);
            }
        }
    };



    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // inflate
        View root = inflater.inflate(R.layout.controls, container, false);

        // get shared preferences
        sharedPreferences = getActivity().getSharedPreferences("Shared Preferences",
                Context.MODE_PRIVATE);



        // variable initialization
        moveForwardImageBtn = Home.getUpBtn();
        turnRightImageBtn = Home.getRightBtn();
        moveBackImageBtn = Home.getDownBtn();
        turnLeftImageBtn = Home.getLeftBtn();
        turnbleftImageBtn = Home.getbLeftBtn();
        turnbrightImageBtn = Home.getbRightBtn();
        exploreTimeTextView = root.findViewById(R.id.exploreTimeTextView2);
        fastestTimeTextView = root.findViewById(R.id.fastestTimeTextView2);
        exploreButton = root.findViewById(R.id.exploreToggleBtn2);
        fastestButton = root.findViewById(R.id.fastestToggleBtn2);
        exploreResetButton = root.findViewById(R.id.exploreResetImageBtn2);
        fastestResetButton = root.findViewById(R.id.fastestResetImageBtn2);
        robotStatusTextView = Home.getRobotStatusTextView();
        fastestTimer = 0;
        exploreTimer = 0;
        //startSend = root.findViewById(R.id.startSend); //just added, need to test

        gridMap = Home.getGridMap();

        // Button Listener
        /*
        moveForwardImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked moveForwardImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("forward");
                    Home.refreshLabel();    // update x and y coordinate displayed
                    // display different statuses depending on validity of robot action
                    if (gridMap.getValidPosition()){
                        updateStatus("moving forward");}
                    else {
                        Home.printMessage(Target.RPI, "Obstacle, invalid position");
                        updateStatus("Unable to move forward");
                    }

                    Home.printMessage(Target.STM, "FW010");
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting moveForwardImageBtn");
            }
        });
        */



        moveForwardImageBtn.setOnTouchListener(new View.OnTouchListener() {
            private final Handler handler = new Handler();
            private final Runnable moveForwardRunnable = new Runnable() {
                @Override
                public void run() {
                    if (gridMap.getCanDrawRobot()) {
                        showLog("Holding moveForwardImageBtn");
                        gridMap.moveRobot("forward");
                        Home.refreshLabel();

                        if (gridMap.getValidPosition()) {
                            updateStatus("moving forward");
                        } else {
                            Home.printMessage(Target.RPI, "Obstacle, invalid position");
                            updateStatus("Unable to move forward");
                        }

                        Home.printMessage(Target.STM, "FW010");
                        handler.postDelayed(this, (1000/3)); // Keep moving forward every (1000/3)ms
                    } else {
                        updateStatus("Please press 'SET START POINT'");
                    }
                }
            };

            @Override
            public boolean onTouch(View v, MotionEvent event) {  // Ensure correct method override
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    showLog("Pressed moveForwardImageBtn");
                    isForwardButtonHeld = true;
                    isBackwardButtonHeld = false;
                    handler.post(moveForwardRunnable);
                    return true;
                } else if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                    showLog("Released moveForwardImageBtn");
                    // isForwardButtonHeld = false;
                    // isBackwardButtonHeld = true;
                    handler.removeCallbacks(moveForwardRunnable);
                    return true;
                }
                return false;
            }
        });


        turnRightImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked turnRightImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("right");
                    Home.refreshLabel();
                    // forward clockwise 90, i.e. turning right
                    Home.printMessage(Target.STM, "FC090");
//                    showLog("test");
                    System.out.println(Arrays.toString(gridMap.getCurCoord()));
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting turnRightImageBtn");
            }
        });



        turnbrightImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked turnbRightImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("backright");
                    Home.refreshLabel();
                    // Backwards Clockwise, turning back right
                    Home.printMessage(Target.STM, "BC090");
                    System.out.println(Arrays.toString(gridMap.getCurCoord()));
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting turnbRightImageBtn");
            }
        });

        /*
        moveBackImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked moveBackwardImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("back");
                    Home.refreshLabel();
                    if (gridMap.getValidPosition())
                        updateStatus("moving backward");
                    else
                        updateStatus("Unable to move backward");
                    Home.printMessage(Target.STM, "BW010");
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting moveBackwardImageBtn");
            }
        });
         */

        moveBackImageBtn.setOnTouchListener(new View.OnTouchListener() {
            private final Handler handler = new Handler();
            private final Runnable moveBackwardRunnable = new Runnable() {
                @Override
                public void run() {
                    if (gridMap.getCanDrawRobot()) {
                        showLog("Holding moveBackImageBtn");
                        gridMap.moveRobot("back");
                        Home.refreshLabel();

                        if (gridMap.getValidPosition()) {
                            updateStatus("moving backward");
                        } else {
                            updateStatus("Unable to move backward");
                        }

                        Home.printMessage(Target.STM, "BW010");
                        handler.postDelayed(this, (1000/3)); // Repeat every (1000/3)ms
                    } else {
                        updateStatus("Please press 'SET START POINT'");
                    }
                }
            };

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        showLog("Pressed moveBackImageBtn");
                        isForwardButtonHeld = false;
                        isBackwardButtonHeld = true;
                        handler.post(moveBackwardRunnable);
                        return true;

                    case MotionEvent.ACTION_UP:
                    case MotionEvent.ACTION_CANCEL:
                        showLog("Released moveBackImageBtn");
                        // isBackwardButtonHeld = false;
                        handler.removeCallbacks(moveBackwardRunnable);
                        return true;
                }
                return false;
            }
        });

        turnLeftImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked turnLeftImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("left");
                    Home.refreshLabel();
                    updateStatus("turning left");
                    // Forward anticlockwise; left turn
                    Home.printMessage(Target.STM, "FA090");
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting turnLeftImageBtn");
            }
        });
        turnbleftImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showLog("Clicked turnbLeftImageBtn");
                if (gridMap.getCanDrawRobot()) {
                    gridMap.moveRobot("backleft");
                    Home.refreshLabel();
                    updateStatus("turning left");
                    // Backwards Anticlockwise (Left)
                    Home.printMessage(Target.STM, "BA090");
                }
                else
                    updateStatus("Please press 'SET START POINT'");
                showLog("Exiting turnbLeftImageBtn");
            }
        });

        // Start Task 1 challenge
        exploreButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showLog("Clicked Task 1 Btn (exploreToggleBtn)");
                ToggleButton exploreToggleBtn = (ToggleButton) v;

                if (exploreToggleBtn.getText().equals("TASK 1 START")) {
                    showToast("Task 1 timer stop!");
                    robotStatusTextView.setText("Task 1 Stopped");
                    timerHandler.removeCallbacks(timerRunnableExplore);
                }
                else if (exploreToggleBtn.getText().equals("STOP")) {
                    // Get String value that represents obstacle configuration
                    String msg = gridMap.getObstacles();
                    // Send this String over via BT
                    //Home.printCoords(msg);
                    //Send BEGIN to the robot
//                    Home.printMessage(Target.RPI, "Begin Task 1"); //send a string "BEGIN" to the RPI
                    String obstacleList = gridMap.getObstacleForAlgo();
                    Log.d("DEBUG", obstacleList);
                    Home.printMessage(Target.ALGO, obstacleList);
                    // Start timer
                    Home.stopTimerFlag = false;
                    showToast("Task 1 timer start!");

                    robotStatusTextView.setText("Task 1 Started");
                    exploreTimer = System.currentTimeMillis();
                    timerHandler.postDelayed(timerRunnableExplore, 0);
                }
                else {
                    showToast("Else statement: " + exploreToggleBtn.getText());
                }
                showLog("Exiting exploreToggleBtn");
            }
        });


        //Start Task 2 Challenge Timer
        fastestButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showLog("Clicked Task 2 Btn (fastestToggleBtn)");
                ToggleButton fastestToggleBtn = (ToggleButton) v;
                if (fastestToggleBtn.getText().equals("TASK 2 START")) {
                    showToast("Task 2 timer stop!");
                    robotStatusTextView.setText("Task 2 Stopped");
                    timerHandler.removeCallbacks(timerRunnableFastest);
                }
                else if (fastestToggleBtn.getText().equals("STOP")) {
                    showToast("Task 2 timer start!");
                    Home.printMessage(Target.STM, "TA001"); //send a string "BEGIN" to the RPI
                    Home.stopWk9TimerFlag = false;
                    robotStatusTextView.setText("Task 2 Started");
                    fastestTimer = System.currentTimeMillis();
                    timerHandler.postDelayed(timerRunnableFastest, 0);
                }
                else
                    showToast(fastestToggleBtn.getText().toString());
                showLog("Exiting fastestToggleBtn");
            }
        });

        exploreResetButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showLog("Clicked exploreResetImageBtn");
                showToast("Resetting exploration time...");
                exploreTimeTextView.setText("00:00");
                robotStatusTextView.setText("Not Available");
                if(exploreButton.isChecked())
                    exploreButton.toggle();
                timerHandler.removeCallbacks(timerRunnableExplore);
                showLog("Exiting exploreResetImageBtn");
            }
        });

        fastestResetButton.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View view) {
                showLog("Clicked fastestResetImgBtn");
                showToast("Resetting Fastest Time...");
                fastestTimeTextView.setText("00:00");
                robotStatusTextView.setText("Fastest Car Finished");
                if(fastestButton.isChecked()){
                    fastestButton.toggle();
                }
                timerHandler.removeCallbacks(timerRunnableFastest);
                showLog("Exiting fastestResetImgBtn");
            }
        });

        /*
        startSend.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View view) {
                showLog("Clicked startSendBtn");
                showToast("Sending BEGIN to robot...");
                exploreButton.toggle();
                if (exploreButton.getText().equals("WK8 START")) {
                    showToast("Auto Movement/ImageRecog timer stop!");
                    robotStatusTextView.setText("Auto Movement Stopped");
                    timerHandler.removeCallbacks(timerRunnableExplore);
                }
                else if (exploreButton.getText().equals("STOP")) {
                    // Get String value that represents obstacle configuration
                    String msg = gridMap.getObstacles();
                    // Send this String over via BT
                    //Home.printCoords(msg);
                    // Start timer
                    Home.stopTimerFlag = false;
                    showToast("Auto Movement/ImageRecog timer start!");

                    robotStatusTextView.setText("Auto Movement Started");
                    exploreTimer = System.currentTimeMillis();
                    timerHandler.postDelayed(timerRunnableExplore, 0);
                }
                //ok
                Home.printMessage("BEGIN"); //send a string "BEGIN" to the RPI
                showLog("Exiting startSend");
            }
        });
         */

        return root;
    }



    private static void showLog(String message) {
        Log.d(TAG, message);
    }

    private void showToast(String message) {
        Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onDestroy(){
        super.onDestroy();
    }

    private void updateStatus(String message) {
        Toast toast = Toast.makeText(getContext(), message, Toast.LENGTH_SHORT);
        toast.setGravity(Gravity.TOP,0, 0);
        toast.show();
    }
}