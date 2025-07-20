package mad.s20195115.myapplication;

import android.content.Intent;
import android.content.pm.ResolveInfo;
import android.net.Uri;
import android.os.Bundle;
import android.speech.RecognizerIntent;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MainActivity";
    private MqttClient mqttClient;
    private static final String BROKER_URL = "tcp://10.123.79.8:1883"; // MQTT broker address
    private static final String CLIENT_ID = "AndroidRobotController";
    private static final String TOPIC_GOAL = "robot/goal";

    private Spinner locationSpinner;
    private TextView voiceResult;
    private TextView connectionStatus;
    private Button sendGoalButton;
    private Button voiceInputButton;
    private Button stopButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Find UI elements
        locationSpinner = findViewById(R.id.locationSpinner);
        voiceResult = findViewById(R.id.voiceResult);
        connectionStatus = findViewById(R.id.connectionStatus);
        sendGoalButton = findViewById(R.id.sendGoalButton);
        voiceInputButton = findViewById(R.id.voiceInputButton);
        stopButton = findViewById(R.id.stopButton);

        // Create spinner adapter with standard Android layouts
        ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(
                this,
                R.array.locations,
                android.R.layout.simple_spinner_item
        );
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        locationSpinner.setAdapter(adapter);

        // Set up MQTT connection
        connectToMqttBroker();

        // Set up accessibility features and button clicks
        setupAccessibility();
    }

    private void setupAccessibility() {
        // Set accessibility properties for dynamic content
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.KITKAT) {
            voiceResult.setAccessibilityLiveRegion(View.ACCESSIBILITY_LIVE_REGION_POLITE);
            connectionStatus.setAccessibilityLiveRegion(View.ACCESSIBILITY_LIVE_REGION_POLITE);
        }

        // Set up button click listeners with accessibility announcements
        sendGoalButton.setOnClickListener(view -> {
            String destination = locationSpinner.getSelectedItem().toString();
            sendNavigationGoal(destination);
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                view.announceForAccessibility("Navigating to " + destination);
            }
        });

        voiceInputButton.setOnClickListener(view -> {
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                view.announceForAccessibility("Starting voice recognition");
            }
            if (getSpeechRecognitionAvailability()) {
                startVoiceRecognition();
            } else {
                showManualLocationDialog();
            }
        });

        stopButton.setOnClickListener(view -> {
            stopNavigation();
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                view.announceForAccessibility("Navigation stopped");
            }
        });

        // Make spinner announce selection changes
        locationSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String selected = parent.getItemAtPosition(position).toString();
                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                    parent.announceForAccessibility("Selected destination: " + selected);
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // Do nothing
            }
        });
    }

    private void connectToMqttBroker() {
        connectionStatus.setText("Connection Status: Connecting...");
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
            connectionStatus.announceForAccessibility("Connecting to robot control system");
        }
        try {
            mqttClient = new MqttClient(BROKER_URL, CLIENT_ID, new MemoryPersistence());
            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);
            options.setUserName("mouser");
            options.setPassword("m0user".toCharArray());

            options.setConnectionTimeout(60);
            options.setKeepAliveInterval(60);

            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.e(TAG, "Connection to MQTT broker lost", cause);
                    runOnUiThread(() -> {
                        connectionStatus.setText("Connection Status: Disconnected");
                        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                            connectionStatus.announceForAccessibility("Connection to robot lost");
                        }
                        Toast.makeText(MainActivity.this,
                                "Connection to robot lost", Toast.LENGTH_LONG).show();
                    });
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    // Not expecting messages from the broker in this app
                    Log.d(TAG, "Message received: " + new String(message.getPayload()));
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    // Message delivery completed
                    Log.d(TAG, "Message delivered");
                }
            });

            mqttClient.connect(options);
            connectionStatus.setText("Connection Status: Connected");
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Connected to robot control system");
            }
            Toast.makeText(this, "Connected to robot control system", Toast.LENGTH_SHORT).show();

        } catch (MqttException e) {
            Log.e(TAG, "Error setting up MQTT client", e);
            connectionStatus.setText("Connection Status: Failed - " + e.getMessage());
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Connection to robot failed");
            }
            Toast.makeText(this, "Error connecting to robot: " + e.getMessage(),
                    Toast.LENGTH_LONG).show();
        }
    }

    private void sendNavigationGoal(String location) {
        // Define location coordinates
        JSONObject goalMsg = new JSONObject();
        try {
            goalMsg.put("action", "navigate");
            goalMsg.put("location", location);
            goalMsg.put("coordinates", createLocationCoordinates(location));

            // Send goal via MQTT
            publishMqttMessage(TOPIC_GOAL, goalMsg.toString());
            Toast.makeText(this, "Navigating to " + location, Toast.LENGTH_LONG).show();
            voiceResult.setText("Robot is going to: " + location);
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                voiceResult.announceForAccessibility("Robot is going to " + location);
            }
        } catch (JSONException e) {
            Log.e(TAG, "Error creating JSON message", e);
            Toast.makeText(this, "Error creating message: " + e.getMessage(),
                    Toast.LENGTH_LONG).show();
        }
    }

    private JSONObject createLocationCoordinates(String location) throws JSONException {
        // Predefined locations
        double x = 0.0, y = 0.0, yaw = 0.0;
        switch (location.toLowerCase()) {
            case "study_room":
                x = -0.725; y = 0.099; yaw = 0.00226;
                break;
            case "toilet":
                x = 1.05; y = -0.187; yaw = 0.000485;
                break;
            case "bedroom":
                x = 0.615; y = -2.81; yaw = -0.0018;
                break;
            case "dining_room":
                x = 3.69; y = 1.87; yaw = 0.000647;
                break;
            case "living_room":
                x = 0.24; y = 1.78; yaw = 0.00119;
                break;
            default:
                Toast.makeText(this, "Unknown location: " + location, Toast.LENGTH_LONG).show();
        }

        // Build coordinates JSON object
        JSONObject coordinates = new JSONObject();
        coordinates.put("x", x);
        coordinates.put("y", y);
        coordinates.put("yaw", yaw);

        return coordinates;
    }

    private void stopNavigation() {
        JSONObject cancelMsg = new JSONObject();
        try {
            cancelMsg.put("action", "cancel");

            // Send cancel command via MQTT
            publishMqttMessage(TOPIC_GOAL, cancelMsg.toString());
            Toast.makeText(this, "Navigation canceled", Toast.LENGTH_LONG).show();
            voiceResult.setText("Robot navigation stopped");
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                voiceResult.announceForAccessibility("Robot navigation stopped");
            }
        } catch (JSONException e) {
            Log.e(TAG, "Error creating cancel message", e);
            Toast.makeText(this, "Error creating cancel message: " + e.getMessage(),
                    Toast.LENGTH_LONG).show();
        }
    }

    private void publishMqttMessage(String topic, String payload) {
        if (mqttClient != null && mqttClient.isConnected()) {
            try {
                MqttMessage message = new MqttMessage(payload.getBytes());
                message.setQos(1);
                mqttClient.publish(topic, message);
                Log.d(TAG, "Published message to topic: " + topic);
            } catch (MqttException e) {
                Log.e(TAG, "Error publishing MQTT message", e);
                Toast.makeText(this, "Error sending command to robot: " + e.getMessage(),
                        Toast.LENGTH_LONG).show();
            }
        } else {
            Toast.makeText(this, "Robot control system not connected! Attempting to reconnect...",
                    Toast.LENGTH_LONG).show();
            connectionStatus.setText("Connection Status: Reconnecting...");
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Reconnecting to robot control system");
            }
            // Try to reconnect
            connectToMqttBroker();
        }
    }

    // Helper method to check if speech recognition is available
    private boolean getSpeechRecognitionAvailability() {
        List<ResolveInfo> activities = getPackageManager().queryIntentActivities(
                new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH), 0);

        if (activities.isEmpty()) {
            Log.w(TAG, "No speech recognition services found on device");
            return false;
        }

        for (ResolveInfo info : activities) {
            Log.d(TAG, "Found speech recognition service: " + info.activityInfo.packageName);
        }

        return true;
    }

    private void startVoiceRecognition() {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Say a destination...");
        intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 5);

        try {
            startActivityForResult(intent, 1);
        } catch (Exception e) {
            Log.e(TAG, "Error starting voice recognition", e);
            Toast.makeText(this, "Error starting voice recognition. Using manual selection instead.",
                    Toast.LENGTH_LONG).show();
            // Fall back to manual dialog
            showManualLocationDialog();
        }
    }

    private void showManualLocationDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Select a Location");

        final String[] locations = getResources().getStringArray(R.array.locations);

        builder.setItems(locations, (dialog, which) -> {
            String selectedLocation = locations[which];
            sendNavigationGoal(selectedLocation);
        });

        builder.setNegativeButton("Cancel", (dialog, which) -> dialog.dismiss());
        AlertDialog dialog = builder.create();
        dialog.show();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == 1) {
            if (resultCode == RESULT_OK && data != null) {
                ArrayList<String> results = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
                String voiceInput = results.get(0).toLowerCase();
                voiceResult.setText("Voice Result: " + voiceInput);

                // Check if voice input contains a recognized location
                if (voiceInput.contains("study") || voiceInput.contains("study room")) {
                    sendNavigationGoal("study_room");
                } else if (voiceInput.contains("toilet") || voiceInput.contains("bathroom")) {
                    sendNavigationGoal("toilet");
                } else if (voiceInput.contains("bedroom") || voiceInput.contains("bed room")) {
                    sendNavigationGoal("bedroom");
                } else if (voiceInput.contains("dining") || voiceInput.contains("dining room")) {
                    sendNavigationGoal("dining_room");
                } else if (voiceInput.contains("living") || voiceInput.contains("living room")) {
                    sendNavigationGoal("living_room");
                } else {
                    Toast.makeText(this, "Unrecognized location: " + voiceInput, Toast.LENGTH_LONG).show();
                    voiceResult.setText("Unrecognized location: " + voiceInput);
                    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                        voiceResult.announceForAccessibility("Unrecognized location: " + voiceInput + ". Please try again or use the spinner to select a location.");
                    }

                    // Show manual selection dialog as fallback
                    showManualLocationDialog();
                }
            } else {
                // Voice recognition failed or was cancelled
                voiceResult.setText("Voice recognition failed or was cancelled");
                showManualLocationDialog();
            }
        }
    }

    @Override
    protected void onDestroy() {
        if (mqttClient != null && mqttClient.isConnected()) {
            try {
                mqttClient.disconnect();
            } catch (MqttException e) {
                Log.e(TAG, "Error disconnecting MQTT client", e);
            }
        }
        super.onDestroy();
    }
}