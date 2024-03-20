// Function to request a device
async function requestDevice(BTName) {
    try {
        const device = await navigator.bluetooth.requestDevice({
            filters: [{ name: BTName }],
        });
        return device;
    } catch (error) {
        console.error("Bluetooth device request failed", error);
        throw error;
    }
}

// Function to connect to a device
async function connectToDevice(device) {
    try {
        const server = await device.gatt.connect();
        return server;
    } catch (error) {
        console.error("Bluetooth device connection failed", error);
        throw error;
    }
}

// Function to get all services
async function getServices(server) {
    try {
        const services = await server.getPrimaryServices();
        return services;
    } catch (error) {
        console.error("Failed to get services", error);
        throw error;
    }
}

//function to read a characteristic value
async function readCharacteristicValue(characteristic) {
    try {
        const value = await characteristic.readValue();
        // this was the trick!
        console.log(`Characteristic value: ${value.getUint8(0)} `);
        return value;
    } catch (error) {
        console.error("Failed to read characteristic value", error);
        throw error;
    }
}

// Function to get all the GATT characteristics from a service and log them to the console
async function getCharacteristics(service) {
    try {
        const characteristics = await service.getCharacteristics();
        for (let characteristic of characteristics) {
            console.log(`Characteristic: ${characteristic.uuid} = ${characteristic.value}`);
            console.log(characteristic);
            readCharacteristicValue(characteristic);
            // value=readCharacteristicValue(characteristic.uuid); 
            // console.log(`Characteristic value: ${value}`)
        }
    } catch (error) {
        console.error("Failed to get characteristics", error);
    }
}

//function that returns the characteristic value for a GATT characteristic and log it to the console
// async function readCharacteristicValue(characteristic) {
//     try {
//         const value = await characteristic.value.read();
//         return value;
//     } catch (error) {
//         console.error("Failed to read characteristic value", error);
//         throw error;
//     }
// }

// Function to enumerate all services
async function enumerateAllGattServices(BTName) {
    try {
        const device = await requestDevice(BTName);
        const server = await connectToDevice(device);
        const services = await getServices(server);

        for (let service of services) {
            console.log(`Service: ${service.uuid}`);
            // call the following function every 10 seconds to read the characteristic values
            let interval = setInterval(getCharacteristics, 10000, service);
            getCharacteristics(service);
        }
    } catch (error) {
        console.error("Failed to enumerate all GATT services", error);
    }
}

// Enumerate all GATT services  
function enumerate(BTName) {
    enumerateAllGattServices(BTName);
}