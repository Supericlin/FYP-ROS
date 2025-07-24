package mad.s20195115.myapplication;

import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ResolveInfo;
import android.net.Uri;
import android.os.Bundle;
import android.speech.RecognizerIntent;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
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
    private static final String CLIENT_ID = "AndroidRobotController";
    private static final String TOPIC_GOAL = "robot/control";
    private static final String TOPIC_STATUS = "robot/status";
    
    // MQTT settings (will be loaded from SharedPreferences)
    private String brokerAddress = "192.168.0.224";
    private String brokerPort = "1883";
    private String username = "mouser";
    private String password = "m0user";

    // Connection state management
    private boolean isConnecting = false;
    private boolean isConnected = false;
    private int reconnectAttempts = 0;
    private static final int MAX_RECONNECT_ATTEMPTS = 3;
    private static final long RECONNECT_DELAY_MS = 5000; // 5 seconds
    private Toast currentToast = null;

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

        // Set up toolbar
        androidx.appcompat.widget.Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

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

        // Load MQTT settings
        loadMqttSettings();
        
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

    private void loadMqttSettings() {
        SharedPreferences prefs = getSharedPreferences("MQTTSettings", MODE_PRIVATE);
        brokerAddress = prefs.getString("broker_address", "192.168.0.224");
        brokerPort = prefs.getString("broker_port", "1883");
        username = prefs.getString("username", "mouser");
        password = prefs.getString("password", "m0user");
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == R.id.action_settings) {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivityForResult(intent, 100); // Use request code 100 for settings
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    public void openSettings(View view) {
        Intent intent = new Intent(this, SettingsActivity.class);
        startActivityForResult(intent, 100);
    }

    private void showToast(String message, int duration) {
        // Cancel any existing toast to prevent stacking
        if (currentToast != null) {
            currentToast.cancel();
        }
        currentToast = Toast.makeText(this, message, duration);
        currentToast.show();
    }

    private void connectToMqttBroker() {
        if (isConnecting) {
            Log.d(TAG, "Already attempting to connect, skipping...");
            return;
        }

        isConnecting = true;
        connectionStatus.setText("Connection Status: Connecting...");
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
            connectionStatus.announceForAccessibility("Connecting to robot control system");
        }

        try {
            String brokerUrl = "tcp://" + brokerAddress + ":" + brokerPort;
            mqttClient = new MqttClient(brokerUrl, CLIENT_ID, new MemoryPersistence());
            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(false); // Disable automatic reconnect to prevent loops
            options.setUserName(username);
            options.setPassword(password.toCharArray());

            options.setConnectionTimeout(30); // Reduced timeout
            options.setKeepAliveInterval(60);

            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.e(TAG, "Connection to MQTT broker lost", cause);
                    isConnected = false;
                    isConnecting = false;
                    
                    runOnUiThread(() -> {
                        connectionStatus.setText("Connection Status: Disconnected");
                        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                            connectionStatus.announceForAccessibility("Connection to robot lost");
                        }
                        showToast("Connection to robot lost", Toast.LENGTH_SHORT);
                        
                        // Attempt reconnection with delay
                        if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                            reconnectAttempts++;
                            connectionStatus.postDelayed(() -> {
                                Log.d(TAG, "Attempting reconnection " + reconnectAttempts + "/" + MAX_RECONNECT_ATTEMPTS);
                                connectToMqttBroker();
                            }, RECONNECT_DELAY_MS);
                        } else {
                            showToast("Connection failed after " + MAX_RECONNECT_ATTEMPTS + " attempts. Please check settings.", Toast.LENGTH_LONG);
                        }
                    });
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String payload = new String(message.getPayload());
                    Log.d(TAG, "Message received from " + topic + ": " + payload);
                    
                    // Handle status messages
                    if (topic.equals(TOPIC_STATUS)) {
                        runOnUiThread(() -> {
                            handleStatusMessage(payload);
                        });
                    }
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    // Message delivery completed
                    Log.d(TAG, "Message delivered");
                }
            });

            mqttClient.connect(options);
            isConnected = true;
            isConnecting = false;
            reconnectAttempts = 0; // Reset reconnect attempts on successful connection
            
            connectionStatus.setText("Connection Status: Connected");
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Connected to robot control system");
            }
            showToast("Connected to robot control system", Toast.LENGTH_SHORT);
            
            // Subscribe to status topic
            mqttClient.subscribe(TOPIC_STATUS);
            Log.d(TAG, "Subscribed to status topic: " + TOPIC_STATUS);

        } catch (MqttException e) {
            Log.e(TAG, "Error setting up MQTT client", e);
            isConnecting = false;
            connectionStatus.setText("Connection Status: Failed - " + e.getMessage());
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Connection to robot failed");
            }
            showToast("Error connecting to robot: " + e.getMessage(), Toast.LENGTH_SHORT);
            
            // Attempt reconnection with delay
            if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                reconnectAttempts++;
                connectionStatus.postDelayed(() -> {
                    Log.d(TAG, "Attempting reconnection " + reconnectAttempts + "/" + MAX_RECONNECT_ATTEMPTS);
                    connectToMqttBroker();
                }, RECONNECT_DELAY_MS);
            } else {
                showToast("Connection failed after " + MAX_RECONNECT_ATTEMPTS + " attempts. Please check settings.", Toast.LENGTH_LONG);
            }
        }
    }

    private void sendNavigationGoal(String location) {
        // Send simple text command (compatible with your MQTT navigation system)
        publishMqttMessage(TOPIC_GOAL, location);
        showToast("Navigating to " + location, Toast.LENGTH_SHORT);
        voiceResult.setText("Robot is going to: " + location);
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
            voiceResult.announceForAccessibility("Robot is going to " + location);
        }
    }

    private void stopNavigation() {
        // Send simple stop command (compatible with your MQTT navigation system)
        publishMqttMessage(TOPIC_GOAL, "navStop");
        showToast("Navigation canceled", Toast.LENGTH_SHORT);
        voiceResult.setText("Robot navigation stopped");
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
            voiceResult.announceForAccessibility("Robot navigation stopped");
        }
    }

    private void publishMqttMessage(String topic, String payload) {
        if (mqttClient != null && mqttClient.isConnected() && isConnected) {
            try {
                MqttMessage message = new MqttMessage(payload.getBytes());
                message.setQos(1);
                mqttClient.publish(topic, message);
                Log.d(TAG, "Published message to topic: " + topic);
            } catch (MqttException e) {
                Log.e(TAG, "Error publishing MQTT message", e);
                showToast("Error sending command to robot: " + e.getMessage(), Toast.LENGTH_SHORT);
            }
        } else {
            showToast("Robot control system not connected! Attempting to reconnect...", Toast.LENGTH_SHORT);
            connectionStatus.setText("Connection Status: Reconnecting...");
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN) {
                connectionStatus.announceForAccessibility("Reconnecting to robot control system");
            }
            // Reset reconnect attempts and try to reconnect
            reconnectAttempts = 0;
            connectToMqttBroker();
        }
    }

    private void handleStatusMessage(String status) {
        // Update UI based on robot status
        if (status.startsWith("navStatus_")) {
            String navStatus = status.substring(10); // Remove "navStatus_" prefix
            switch (navStatus) {
                case "1":
                    voiceResult.setText("Robot Status: Navigation Started");
                    break;
                case "4":
                    voiceResult.setText("Robot Status: Navigation in Progress");
                    break;
                case "5":
                    voiceResult.setText("Robot Status: Navigation Completed");
                    break;
                case "6":
                    voiceResult.setText("Robot Status: Navigation Canceled");
                    break;
            }
        } else if (status.startsWith("battery_voltage:")) {
            String voltage = status.substring(15); // Remove "battery_voltage:" prefix
            connectionStatus.setText("Battery: " + voltage + "V");
        } else if (status.equals("batt_red")) {
            showToast("Warning: Low Battery!", Toast.LENGTH_SHORT);
            connectionStatus.setText("Battery: LOW");
        } else if (status.equals("FALL_DETECTED")) {
            showToast("ALERT: Fall Detected!", Toast.LENGTH_LONG);
            voiceResult.setText("Robot Status: Fall Detected - Emergency Stop");
        } else if (status.equals("no_pressure")) {
            showToast("Warning: No Pressure Detected!", Toast.LENGTH_SHORT);
            voiceResult.setText("Robot Status: No Pressure - Navigation Stopped");
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
            showToast("Error starting voice recognition. Using manual selection instead.", Toast.LENGTH_SHORT);
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
                if (voiceInput.contains("mcdonald") || voiceInput.contains("mcdonald")) {
                    sendNavigationGoal("mcdonald");
                } else if (voiceInput.contains("toilet") || voiceInput.contains("bathroom") || voiceInput.contains("washroom")) {
                    sendNavigationGoal("toilet");
                } else if (voiceInput.contains("entrance")) {
                    sendNavigationGoal("entrance");
                } else if (voiceInput.contains("watsons")) {
                    sendNavigationGoal("watsons");
                } else if (voiceInput.contains("information_counter") || voiceInput.contains("information center")) {
                    sendNavigationGoal("information_counter");
                } else if (voiceInput.contains("starbucks")) {
                    sendNavigationGoal("starbucks");
                } else {
                    showToast("Unrecognized location: " + voiceInput, Toast.LENGTH_SHORT);
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
        } else if (requestCode == 100) {
            // Settings activity returned
            if (resultCode == RESULT_OK) {
                // Reload settings and reconnect
                loadMqttSettings();
                if (mqttClient != null && mqttClient.isConnected()) {
                    try {
                        mqttClient.disconnect();
                    } catch (MqttException e) {
                        Log.e(TAG, "Error disconnecting MQTT client", e);
                    }
                }
                // Reset connection state
                isConnected = false;
                isConnecting = false;
                reconnectAttempts = 0;
                connectToMqttBroker();
                showToast("MQTT settings updated and reconnected", Toast.LENGTH_SHORT);
            }
        }
    }

    @Override
    protected void onDestroy() {
        // Cancel any existing toast
        if (currentToast != null) {
            currentToast.cancel();
        }
        
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