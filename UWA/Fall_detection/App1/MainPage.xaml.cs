using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Microsoft.Maker.RemoteWiring;
using Microsoft.Maker.Serial;
using Windows.UI.ViewManagement;
using Windows.UI;

namespace App1
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    { 
        public MainPage()
        {
            this.InitializeComponent();
           
            // this.InitArduino(); //Init Arduino connection
        }

        Microsoft.Maker.RemoteWiring.RemoteDevice arduino;
        Microsoft.Maker.Serial.NetworkSerial netWorkSerial;

        public void InitArduino()
        {
            //Establish a network serial connection. change it to the right IP address and port
            //netWorkSerial = new Microsoft.Maker.Serial.NetworkSerial(new Windows.Networking.HostName("172.20.10.3"), 3030);

            //netWorkSerial = new Microsoft.Maker.Serial.NetworkSerial(new Windows.Networking.HostName("192.168.0.106"), 3030);
            netWorkSerial = new Microsoft.Maker.Serial.NetworkSerial(new Windows.Networking.HostName(IPTextBox.Text), (ushort)Convert.ToInt32(PortTextBox.Text));
            //Create Arduino Device
            arduino = new Microsoft.Maker.RemoteWiring.RemoteDevice(netWorkSerial);

            //Attach event handlers
            netWorkSerial.ConnectionEstablished += NetWorkSerial_ConnectionEstablished;
            netWorkSerial.ConnectionFailed += NetWorkSerial_ConnectionFailed;
            netWorkSerial.ConnectionLost += NetWorkSerial_ConnectionLost;

            //subscribe to the ConnectionEstablished event with the name of the function to be called.
            arduino.DeviceReady += MyDeviceReadyCallback;

            arduino.StringMessageReceived += MyDeviceStringMessageReceivedCallback;
            //subscribe to the DigitalPinUpdateEvent with the name of the function to be called.
            //arduino.DigitalPinUpdated += MyDigitalPinUpdateCallback;

            //subscribe to the AnalogPinUpdateEvent with the name of the function to be called.
            arduino.AnalogPinUpdated += MyAnalogPinUpdateCallback;

            System.Diagnostics.Debug.WriteLine("Staring comms");
            //Begin connection
            netWorkSerial.begin(115200, Microsoft.Maker.Serial.SerialConfig.SERIAL_8N1);
        }

        private void NetWorkSerial_ConnectionEstablished()
        {
            ConnectionIndicator.Background = new SolidColorBrush(Windows.UI.Colors.Green);
            System.Diagnostics.Debug.WriteLine("Arduino Connection successful");
            //arduino.pinMode(6, Microsoft.Maker.RemoteWiring.PinMode.OUTPUT); //Set the pin to output
            //arduino.pinMode("A0", PinMode.ANALOG);
            //arduino.pinMode("A1", PinMode.ANALOG);
            //arduino.pinMode("A2", PinMode.ANALOG);
            //turn it to High. The RED LED on Arduino Yun should light up
            //arduino.digitalWrite(6, Microsoft.Maker.RemoteWiring.PinState.HIGH);


        }

        
        private void NetWorkSerial_ConnectionLost(string message)
        {
            ConnectionIndicator.Background = new SolidColorBrush(Windows.UI.Colors.Red);
            //System.Diagnostics.Debug.WriteLine("Arduino Connection Lost: " + message);
            System.Diagnostics.Debug.WriteLine("Retrying Arduino Connection...");
            //netWorkSerial.begin(115200, Microsoft.Maker.Serial.SerialConfig.SERIAL_8N1);
        }

        private void NetWorkSerial_ConnectionFailed(string message)
        {
            ConnectionIndicator.Background = new SolidColorBrush(Windows.UI.Colors.Red);
            System.Diagnostics.Debug.WriteLine("Arduino Connection Failed: " + message);
            System.Diagnostics.Debug.WriteLine("Retrying Arduino Connection...");
            netWorkSerial.begin(115200, Microsoft.Maker.Serial.SerialConfig.SERIAL_8N1);
        }

        public void MyDeviceStringMessageReceivedCallback(string message)
        {
            //System.Diagnostics.Debug.WriteLine("String Received: ["+message+"]");
            var action = Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, new Windows.UI.Core.DispatchedHandler(() => {
                DeviceStatusTextBlock.Text = message;
                if (message.Equals("FALLEN!"))
                    DeviceStatusTextBlock.Foreground = new SolidColorBrush(Windows.UI.Colors.Red);
                else
                    DeviceStatusTextBlock.Foreground = new SolidColorBrush(Windows.UI.Colors.Green);

            }));
            
        }
        //this function will automatically be called when a device connection is established and the device is ready
        //you may think of this like setup() in an Arduino sketch. It is the best place to prepare your
        //Arduino for the logic that the rest of your program will execute
        public void MyDeviceReadyCallback()
        {

            //arduino.pinMode(3, PinMode.OUTPUT);
           // arduino.pinMode(6, PinMode.OUTPUT);
            //set pin 7 to ANALOG mode to automatically receive callbacks when it changes
            arduino.pinMode("A0", PinMode.ANALOG);
            arduino.pinMode("A1", PinMode.ANALOG);
            arduino.pinMode("A2", PinMode.ANALOG);
            //arduino.pinMode(3, PinMode.INPUT);
            /*
            while (true)
            {
                UInt16 val0 = arduino.analogRead("A0");
                UInt16 val1 = arduino.analogRead("A1");
                UInt16 val2 = arduino.analogRead("A2");
                
                //System.Diagnostics.Debug.WriteLine("analog pin A0  is now " + val0.ToString());
                var action = Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, new Windows.UI.Core.DispatchedHandler(() => {
                    A0TextBlock.Text = (val0.ToString());
                    A1TextBlock.Text = (val1.ToString());
                    A2TextBlock.Text = (val2.ToString());
                    
                }));
                
                //arduino.digitalWrite(6, PinState.HIGH);
            }*/

        }

        //this function will automatically be called whenever a digital pin value changes
        /*
        public  void MyDigitalPinUpdateCallback(byte pin, PinState state)
        {
            System.Diagnostics.Debug.WriteLine("Digital pin " + pin + " is now " + (state == PinState.HIGH ? "HIGH" : "LOW" ));
            string text = "";
            if (pin == 3 && state == PinState.HIGH) 
                text = "FALLEN!";
            else
                text = "NORMAL";
            var action = Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, new Windows.UI.Core.DispatchedHandler(() => {
                D3TextBlock.Text = text;

            }));
        }*/

        //this function will automatically be called whenever the analog values are reported. This is usually every few ms.
        public void MyAnalogPinUpdateCallback(string pin, UInt16 value)
        {
            //System.Diagnostics.Debug.WriteLine("Analog pin A" + pin + " is now " + value);
            //A0TextBlock.Text = "" + arduino.analogRead("A0");
            //A1TextBlock.Text = "" + arduino.analogRead("A1");
            //A2TextBlock.Text = "" + arduino.analogRead("A2");
            var action = Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, new Windows.UI.Core.DispatchedHandler(() => {
                if(pin.Equals("A0"))
                 A0TextBlock.Text = (value.ToString());
                else if (pin.Equals("A1"))
                    A1TextBlock.Text = (value.ToString());
                else if (pin.Equals("A2"))
                    A2TextBlock.Text = (value.ToString());

            }));

        }

        private void button_Click(object sender, RoutedEventArgs e)
        {
            arduino.pinMode(6, PinMode.OUTPUT);
            arduino.pinMode(5, PinMode.OUTPUT);
            arduino.digitalWrite(6, PinState.HIGH);
            arduino.digitalWrite(5, PinState.LOW);
        }

        private void buttonOff_Click(object sender, RoutedEventArgs e)
        {
            arduino.pinMode(6, PinMode.OUTPUT);
            arduino.pinMode(5, PinMode.OUTPUT);
            arduino.digitalWrite(4, PinState.LOW);
            arduino.digitalWrite(5, PinState.HIGH);
        }

        private void buttonConnect_Click(object sender, RoutedEventArgs e)
        {
            this.InitArduino(); //Init Arduino connection
        }

        private void buttonDisConnect_Click(object sender, RoutedEventArgs e)
        {
            netWorkSerial.Dispose();
        }
    }
}
