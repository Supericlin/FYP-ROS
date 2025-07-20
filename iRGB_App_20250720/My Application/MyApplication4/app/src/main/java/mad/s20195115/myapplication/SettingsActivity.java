package mad.s20195115.myapplication;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.MenuItem;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

public class SettingsActivity extends AppCompatActivity {
    private EditText brokerAddressEdit;
    private EditText brokerPortEdit;
    private EditText usernameEdit;
    private EditText passwordEdit;
    private Button saveButton;
    private Button testConnectionButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        // Set up toolbar
        androidx.appcompat.widget.Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        // Enable back button in action bar
        if (getSupportActionBar() != null) {
            getSupportActionBar().setDisplayHomeAsUpEnabled(true);
        }

        // Initialize UI elements
        brokerAddressEdit = findViewById(R.id.brokerAddressEdit);
        brokerPortEdit = findViewById(R.id.brokerPortEdit);
        usernameEdit = findViewById(R.id.usernameEdit);
        passwordEdit = findViewById(R.id.passwordEdit);
        saveButton = findViewById(R.id.saveButton);
        testConnectionButton = findViewById(R.id.testConnectionButton);

        // Load current settings
        loadSettings();

        // Set up button listeners
        saveButton.setOnClickListener(v -> saveSettings());
        testConnectionButton.setOnClickListener(v -> testConnection());
    }

    private void loadSettings() {
        SharedPreferences prefs = getSharedPreferences("MQTTSettings", MODE_PRIVATE);
        brokerAddressEdit.setText(prefs.getString("broker_address", "192.168.0.224"));
        brokerPortEdit.setText(prefs.getString("broker_port", "1883"));
        usernameEdit.setText(prefs.getString("username", "mouser"));
        passwordEdit.setText(prefs.getString("password", "m0user"));
    }

    private void saveSettings() {
        String address = brokerAddressEdit.getText().toString().trim();
        String port = brokerPortEdit.getText().toString().trim();
        String username = usernameEdit.getText().toString().trim();
        String password = passwordEdit.getText().toString().trim();

        // Validate inputs
        if (address.isEmpty() || port.isEmpty() || username.isEmpty() || password.isEmpty()) {
            Toast.makeText(this, "Please fill in all fields", Toast.LENGTH_SHORT).show();
            return;
        }

        // Validate port number
        try {
            int portNum = Integer.parseInt(port);
            if (portNum < 1 || portNum > 65535) {
                Toast.makeText(this, "Port must be between 1 and 65535", Toast.LENGTH_SHORT).show();
                return;
            }
        } catch (NumberFormatException e) {
            Toast.makeText(this, "Invalid port number", Toast.LENGTH_SHORT).show();
            return;
        }

        // Save settings
        SharedPreferences prefs = getSharedPreferences("MQTTSettings", MODE_PRIVATE);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putString("broker_address", address);
        editor.putString("broker_port", port);
        editor.putString("username", username);
        editor.putString("password", password);
        editor.apply();

        Toast.makeText(this, "Settings saved successfully", Toast.LENGTH_SHORT).show();
        setResult(RESULT_OK);
        finish();
    }

    private void testConnection() {
        String address = brokerAddressEdit.getText().toString().trim();
        String port = brokerPortEdit.getText().toString().trim();
        String username = usernameEdit.getText().toString().trim();
        String password = passwordEdit.getText().toString().trim();

        if (address.isEmpty() || port.isEmpty() || username.isEmpty() || password.isEmpty()) {
            Toast.makeText(this, "Please fill in all fields first", Toast.LENGTH_SHORT).show();
            return;
        }

        // Show testing message
        Toast.makeText(this, "Testing connection...", Toast.LENGTH_SHORT).show();

        // In a real implementation, you would test the MQTT connection here
        // For now, we'll just show a success message
        Toast.makeText(this, "Connection test completed (simulated)", Toast.LENGTH_SHORT).show();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == android.R.id.home) {
            onBackPressed();
            return true;
        }
        return super.onOptionsItemSelected(item);
    }
} 