const express = require('express');
const app = express();
const path = require('path');
const { exec } = require('child_process');
const bodyParser = require('body-parser');

// Middleware to parse JSON bodies
app.use(bodyParser.json());

// Serve static files from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Serve the index.html file
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Run python script
// Endpoint to run the Python script
app.post('/run-python', (req, res) => {
    const { selectedObject } = req.body;
    console.log('Received data:', selectedObject);

    // Run the Python script

    if (selectedObject === "blue-object") {
        exec(`python3 blue_connector.py`, (error, stdout, stderr) => {
            if (error) {
                console.error(`Error: ${error.message}`);
                return res.status(500).json({ error: error.message });
            }

            if (stderr) {
                console.error(`Stderr: ${stderr}`);
                return res.status(500).json({ error: stderr });
            }

            console.log(`Stdout: ${stdout}`);
            res.status(200).json({ output: stdout });
        });
    } else if (selectedObject === "white-object") {
        exec(`python3 white_connector.py`, (error, stdout, stderr) => {
            if (error) {
                console.error(`Error: ${error.message}`);
                return res.status(500).json({ error: error.message });
            }

            if (stderr) {
                console.error(`Stderr: ${stderr}`);
                return res.status(500).json({ error: stderr });
            }

            console.log(`Stdout: ${stdout}`);
            res.status(200).json({ output: stdout });
        });
    } else if (selectedObject === "release-motors") {
        exec(`python3 Release_motors.py`, (error, stdout, stderr) => {
            if (error) {
                console.error(`Error: ${error.message}`);
                return res.status(500).json({ error: error.message });
            }

            if (stderr) {
                console.error(`Stderr: ${stderr}`);
                return res.status(500).json({ error: stderr });
            }

            console.log(`Stdout: ${stdout}`);
            res.status(200).json({ output: stdout });
        });
    } else {
        exec(`python3 start_demo.py`, (error, stdout, stderr) => {
            if (error) {
                console.error(`Error: ${error.message}`);
                return res.status(500).json({ error: error.message });
            }

            if (stderr) {
                console.error(`Stderr: ${stderr}`);
                return res.status(500).json({ error: stderr });
            }

            console.log(`Stdout: ${stdout}`);
            res.status(200).json({ output: stdout });
        });
    }

});

const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});