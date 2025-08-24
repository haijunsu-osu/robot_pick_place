// PID Control Demo for Electric Servo Motor
// Author: GitHub Copilot

// --- PID Controller Class ---
class PID {
    constructor(kp, ki, kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.reset();
    }
    reset() {
        this.integral = 0;
        this.prevError = 0;
    }
    update(error, dt) {
        this.integral += error * dt;
        const derivative = (error - this.prevError) / dt;
        const output = this.kp * error + this.ki * this.integral + this.kd * derivative;
        this.prevError = error;
        return output;
    }
    setParams(kp, ki, kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}

// --- Servo Motor Model (1st order lag) ---
class ServoMotor {
    constructor() {
        this.position = 0; // degrees
        this.velocity = 0;
        this.maxSpeed = 180; // deg/sec
        this.timeConstant = 0.15; // seconds (motor lag)
    }
    update(input, dt) {
        // input: control signal (target speed, -1 to 1)
        // Simple first-order lag model
        const targetSpeed = Math.max(-1, Math.min(1, input)) * this.maxSpeed;
        // Exponential approach to target speed
        this.velocity += (targetSpeed - this.velocity) * (1 - Math.exp(-dt / this.timeConstant));
        this.position += this.velocity * dt;
    }
    reset() {
        this.position = 0;
        this.velocity = 0;
    }
}

// --- UI and Simulation ---
const canvas = document.getElementById('plot');
const ctx = canvas.getContext('2d');
const pSlider = document.getElementById('p-slider');
const iSlider = document.getElementById('i-slider');
const dSlider = document.getElementById('d-slider');
const targetSlider = document.getElementById('target-slider');
const resetBtn = document.getElementById('reset-btn');
const pVal = document.getElementById('p-val');
const iVal = document.getElementById('i-val');
const dVal = document.getElementById('d-val');
const targetVal = document.getElementById('target-val');
const connectBtn = document.getElementById('connect-btn');
const serialStatus = document.getElementById('serial-status');
const sendPidBtn = document.getElementById('send-pid-btn');
const clearRealBtn = document.getElementById('clear-real-btn');

let pid = new PID(parseFloat(pSlider.value), parseFloat(iSlider.value), parseFloat(dSlider.value));
let servo = new ServoMotor();
let target = parseFloat(targetSlider.value);
let time = 0;
let data = [];
let realData = [];

// --- Serial Communication ---
let port = null;
let reader = null;
let keepReading = false;

async function connectSerial() {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        serialStatus.textContent = 'Connected to Arduino.';
        keepReading = true;
        readSerialLoop();
    } catch (e) {
        serialStatus.textContent = 'Connection failed.';
    }
}

async function readSerialLoop() {
    if (!port) return;
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
    const reader = textDecoder.readable.getReader();
    let buffer = '';
    try {
        while (keepReading) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) {
                buffer += value;
                let lines = buffer.split('\n');
                buffer = lines.pop();
                for (let line of lines) {
                    // Expecting CSV: time,angle,speed\n
                    let parts = line.trim().split(',');
                    if (parts.length >= 2) {
                        let t = parseFloat(parts[0]);
                        let pos = parseFloat(parts[1]);
                        realData.push({ t, pos });
                        if (realData.length > 500) realData.shift();
                    }
                }
            }
        }
    } catch (e) {
        serialStatus.textContent = 'Serial read error.';
    } finally {
        reader.releaseLock();
    }
}

function sendPIDToArduino() {
    if (!port || !port.writable) {
        serialStatus.textContent = 'Not connected.';
        return;
    }
    const writer = port.writable.getWriter();
    // Send as: P,I,D,TARGET\n
    const msg = `${pSlider.value},${iSlider.value},${dSlider.value},${targetSlider.value}\n`;
    writer.write(new TextEncoder().encode(msg));
    writer.releaseLock();
    serialStatus.textContent = 'PID and target sent.';
}

function clearRealData() {
    realData = [];
}

connectBtn.addEventListener('click', connectSerial);
sendPidBtn.addEventListener('click', sendPIDToArduino);
clearRealBtn.addEventListener('click', clearRealData);

function resetSim() {
    pid.reset();
    servo.reset();
    time = 0;
    data = [];
}

function updateParams() {
    pid.setParams(parseFloat(pSlider.value), parseFloat(iSlider.value), parseFloat(dSlider.value));
    pVal.textContent = pSlider.value;
    iVal.textContent = iSlider.value;
    dVal.textContent = dSlider.value;
    target = parseFloat(targetSlider.value);
    targetVal.textContent = targetSlider.value;
}

[pSlider, iSlider, dSlider, targetSlider].forEach(slider => {
    slider.addEventListener('input', () => {
        updateParams();
        resetSim();
    });
});
resetBtn.addEventListener('click', () => {
    resetSim();
});

function drawPlot() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Axes
    ctx.strokeStyle = '#bbb';
    ctx.beginPath();
    ctx.moveTo(40, 10);
    ctx.lineTo(40, 290);
    ctx.lineTo(490, 290);
    ctx.stroke();
    // Target line
    ctx.strokeStyle = 'red';
    ctx.beginPath();
    let y = 290 - (target / 180) * 260;
    ctx.moveTo(40, y);
    ctx.lineTo(490, y);
    ctx.stroke();
    // Motor position curve (simulation)
    ctx.strokeStyle = 'blue';
    ctx.beginPath();
    for (let i = 0; i < data.length; i++) {
        let t = 40 + (i / 500) * 450;
        let pos = 290 - (data[i].pos / 180) * 260;
        if (i === 0) ctx.moveTo(t, pos);
        else ctx.lineTo(t, pos);
    }
    ctx.stroke();
    // Real motor data curve
    if (realData.length > 1) {
        ctx.strokeStyle = 'green';
        ctx.beginPath();
        for (let i = 0; i < realData.length; i++) {
            let t = 40 + (i / 500) * 450;
            let pos = 290 - (realData[i].pos / 180) * 260;
            if (i === 0) ctx.moveTo(t, pos);
            else ctx.lineTo(t, pos);
        }
        ctx.stroke();
    }
    // Labels
    ctx.fillStyle = '#333';
    ctx.font = '12px Arial';
    ctx.fillText('0°', 10, 290);
    ctx.fillText('180°', 5, 35);
    ctx.fillText('Time (s)', 230, 305);
}

function stepSim() {
    const dt = 0.02; // 20 ms
    let error = target - servo.position;
    let control = pid.update(error, dt);
    // Normalize control to -1..1 for servo input
    let controlNorm = Math.max(-1, Math.min(1, control / 90));
    servo.update(controlNorm, dt);
    time += dt;
    data.push({ t: time, pos: servo.position });
    if (data.length > 500) data.shift();
    drawPlot();
    requestAnimationFrame(stepSim);
}

// Initialize
updateParams();
resetSim();
stepSim();
