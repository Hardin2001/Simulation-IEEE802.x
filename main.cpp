#include <iostream>
#include <string>
#include "stdio.h"
#include <queue>
#include <random>
#include <vector>

using namespace std;

/*
 *  Editable constants for simulation modification
 *  DEBUG_MODE:     (0:Removes Simulation Output, 1:Includes Simulation output)
 *  TOTAL_EVENTS:   100,000 by default
 *  MAX_BUFFER:     (integer: Real size buffer, INFINITY:Infinite size buffer)
 *  LAMBDA:         Arrival Rate Constant
 *  MU:             Departure Rate Constant
 */
#define DEBUG_MODE 0 // 0:faster runetime, 1:full simulation output
#define TOTAL_EVENTS 100000
#define MAX_BUFFER INFINITY
#define LAMBDA 0.5 //Frames generated per second by each host
#define T 100

#define SIFS 0.00005 //sec        // 0.05 msec
#define DIFS 0.0001 //sec         // 0.1  msec
#define MIN_INTERVAL 0.00001 //sec// 0.01 msec
#define LENGTH_LAMBDA 0.0001
#define TRANSFER_RATE 10000000//

 /*
  *  Non-modifiable constnats for simulation
  *  GEN_NEW: Default value for newEvent(newTime) to generate a new event time.
  */
#define GEN_NEW -DBL_MAX
#define ARRIVAL 'A'
#define DEPARTURE 'D'
#define SEND 'S'
#define WAIT 'W'
#define ACK 'K'
#define TIMER 'T'

  /*
   *  Debugging output macros, used for multiple runetime modes
   *  SYS_OUT: Toggled by DEBUG_MODE (0:Off, 1:On). System runtime output
   *  SIM_OUT: Macro to simplify output to system after running simulation
   */
#define SYS_OUT(X) do {if (DEBUG_MODE) {cout << X << endl;}} while(0)
#define SIM_OUT(X) do {cout << X << endl;} while(0)

const int g_total_events = TOTAL_EVENTS;
const double g_max_buffer = MAX_BUFFER;

double g_Lambda;
int g_Num_Hosts;

double g_Time = 0.0;
double g_Transmission_Time = 0.0;
double g_TimeStart = 0.0;
double g_StartIdle = 0.0;
double g_StopIdle = 0.0;
double g_FreezeStart = 0.0;
double g_FreezeStop = 0.0;
double g_Delta_Time = 0.0;
double g_Idle_Time = 0.0;
double g_Queue_Time = 0.0;

int g_channel = 0;
int g_Packets_Dropped = 0;
int g_Packets_Sent = 0;

/*
 *  Packet: Contains data to be placed into buffer and is extracted on process
 *  time:   Time that the packet will spend in processing, used to create a
 *          departure event after dequeued from the buffer.
 */
class Packet {
public:
    bool ack;
    bool channel = false;
    int length;
    int source;
    int dest;
    int detect_busy = 0;
    double time = 0.0;
    double queue_delay = 0.0;
    double transfer_delay= 0.0;
    bool collision = false;

    Packet(bool Ack, int Length, int Source, int Destination, double Time) {
        ack = Ack;
        length = Length;
        source = Source;
        dest = Destination;
        time = Time;
    }
    Packet() {}
    ~Packet() {};

};
/*
 * Event: Contains data pertaining to Arrivaland Departure Events
 * type : 0 : Departure, 1 : Arrival
 * time : Time that the event occurs.
 */
class Event {
public:
    int host;
    double start_time;
    double end_time;
    char type;
    int dest;
    Packet packet;

    Event(int Host, double Start_time, double End_time, char Type, int Dest, Packet Pack) {
        host = Host;
        start_time = Start_time;
        end_time = End_time;
        type = Type;
        dest = Dest;
        packet = Pack;
    }
    ~Event() {};
    //Used for GEL implementation, priority_queue uses std::greater to
    //place event in order by time
    inline bool operator > (const Event& right) const {
        return end_time > right.end_time;
    }

};

class Host {
public:
    queue<Packet> buffer;
    Host() {};
};

queue<Event> g_Gel;
default_random_engine g_GetRand;
vector<Host> g_Hosts;

int generateDestination(int self, int N) {
    int dest = self;
    while (dest == self) {
        dest = rand() % (N - 1);
    }
    return dest;
}
/*
 *  Helper funciton called for new arrival events and packets
 *      Generates a number between 0.0 and 1.0, and plugs this value
 *      into a negative exponential function along with the provided rate
 */
double NegativeExponential(double Rate) {
    uniform_real_distribution<double> dist(0.0, 1.0);
    double u = dist(g_GetRand);
    double retval = (((double)-1 / Rate) * log(1 - u));
    return ((-1 / Rate) * log(1 - u));
}

int CreateNewPacketSize() {
    double d_length = NegativeExponential(LENGTH_LAMBDA);
    return (int)d_length % 1544;
}

double BinaryExponentialBackoff(int n) {
    int bkoff = 0;
    int exp = 0;
    while (bkoff == 0) {
        bkoff = g_GetRand() % T;
    }
    while (exp == 0) {
        exp = g_GetRand() % ((int)pow(2, n));
    }
    return (double)bkoff * (double)exp;
}

/*
 *  Helper function called for every arrival event from g_System.Gel
 *      Generates new arrival event, and either processes a new packet,
 *      enqueues a new packet in the buffer, or drops the packet if the
 *      buffer is at max capacity.
 */
void ProcessArrival(Event Arrival) {
    int source = (int)g_GetRand() % g_Num_Hosts;
    double pack_time = g_Time + NegativeExponential(g_Lambda);
    int destination = generateDestination(source, g_Num_Hosts);

    Packet packet = Packet(false, CreateNewPacketSize(), source, destination, pack_time);
    Event event = Event(source, g_Time, pack_time, ARRIVAL, destination, packet);
    g_Gel.push(event);
    
    g_Hosts[source].buffer.push(packet);
    //If the system is currently not processing any packets.
    if (g_Hosts[source].buffer.size() == 1) {

        if (g_channel == 0) { // Cornercase?
            double time = g_Time + DIFS;
            Event depart = Event(Arrival.packet.source, g_Time, time, DEPARTURE, Arrival.packet.dest, Arrival.packet);
            g_Gel.push(depart);
        }
        else {
            Arrival.packet.detect_busy++;
            double time = g_Time + DIFS + BinaryExponentialBackoff(Arrival.packet.detect_busy);
            Event wait = Event(Arrival.packet.source, g_Time, time, DEPARTURE, Arrival.packet.dest, Arrival.packet);
            g_Gel.push(wait);
        }

    }
    else {
        if (g_Hosts[source].buffer.size() == MAX_BUFFER) {
            g_Packets_Dropped++;
        }
        else{
            g_Hosts[source].buffer.push(Arrival.packet);
        }
    }
}

void UnfreezeTimers() {
    g_FreezeStop = g_Time;
    g_Delta_Time = g_FreezeStop - g_FreezeStart;

    //Go through all Events in g_GEL.

    //If STATE == DEPARTURE or ACK;

        //Increase Stop time by g_Delta_Time  

}

void FreezeTimers() {
    g_StopIdle = g_Time;
    g_Idle_Time += g_StopIdle - g_StartIdle;
    g_FreezeStart = g_Time;
}

void MarkAllCollision() {

    for (int i = 0; i < g_Num_Hosts; i++) {
        if (g_Hosts[i].buffer.front().channel == true) {
            g_Hosts[i].buffer.front().collision = true;
        }
    }
}

/*
 *  Helper function called for every departure event from g_System.Gel
 */
void ProcessDeparture(Event Departure) {
    double delta_Time = Departure.end_time - Departure.start_time;
    Departure.packet.queue_delay += delta_Time;

    //Stay in the first spot in the queue until freed.

    Event send = Event(Departure.host, g_Time, (double)(g_Time + ((Departure.packet.length * 8) / TRANSFER_RATE)), SEND, Departure.dest, Departure.packet);

    if (g_channel == 0) {
        FreezeTimers();
    }

    g_channel++;

    if (g_channel != 1) {
        MarkAllCollision();
        Departure.packet.collision = true;
    }
    g_Gel.push(send);

}

void ProcessSent(Event Sent) {
    double delta_Time = Sent.end_time - Sent.start_time;
    Sent.packet.transfer_delay += delta_Time;

    if (g_channel != 1) {
        MarkAllCollision();
        Sent.packet.collision = true;
    }

    if (g_channel == 0) {
        UnfreezeTimers();
    }

    Event* AckTimer = new Event(Sent.host, g_Time, (double)(g_Time + (1.5*(SIFS)+(64 * 8) / TRANSFER_RATE)), TIMER, Sent.packet.dest, Sent.packet);

    if (g_Hosts[Sent.dest].buffer.size() == 0) {

        if (g_channel == 0) {

            Event sendAck = Event(Sent.packet.source, g_Time, g_Time + SIFS, ACK, Sent.packet.dest, Sent.packet);
            g_Gel.push(sendAck);

        }
        else {
            Sent.packet.detect_busy++;
            double time = g_Time + SIFS + BinaryExponentialBackoff(Sent.packet.detect_busy);
            Event sendAck = Event(Sent.packet.source, g_Time, time, ACK, Sent.packet.dest, Sent.packet);
            g_Gel.push(sendAck);
        }

    }
}

void ProcessAckTimer(Event AckTimer) {
    double delta_Time = AckTimer.end_time - AckTimer.start_time;
    AckTimer.packet.transfer_delay += delta_Time;

    if (AckTimer.packet.time == g_Hosts[AckTimer.host].buffer.front().time) {

        if (g_channel == 0) { // Cornercase?
            double time = g_Time + DIFS;
            Event depart = Event(AckTimer.host, g_Time, time, DEPARTURE, AckTimer.dest, g_Hosts[AckTimer.host].buffer.front());
            g_Gel.push(depart);
        }
        else {
            g_Hosts[AckTimer.host].buffer.front().detect_busy++;
            double time = g_Time + DIFS + BinaryExponentialBackoff(g_Hosts[AckTimer.host].buffer.front().detect_busy);
            Event wait = Event(AckTimer.host, g_Time, time, DEPARTURE, AckTimer.dest, g_Hosts[AckTimer.host].buffer.front());
            g_Gel.push(wait);
        }
    }

}

void ProcessAck(Event Ack) {
    double delta_Time = Ack.end_time - Ack.start_time;
    Ack.packet.transfer_delay += delta_Time;
    
    if ((g_Time < Ack.start_time + (1.5 * (SIFS + (64 * 8) / TRANSFER_RATE)))) {
        Packet done = g_Hosts[Ack.host].buffer.front();
        g_Packets_Sent += done.length + 64;
        g_Transmission_Time += done.transfer_delay;

        g_Hosts[Ack.host].buffer.pop();

        if (g_Hosts[Ack.host].buffer.size() > 0) {

            if (g_channel == 0) { 
                double time = g_Time + DIFS;
                Event depart = Event(Ack.host, g_Time, time, DEPARTURE, g_Hosts[Ack.host].buffer.front().dest, g_Hosts[Ack.host].buffer.front());
                g_Gel.push(depart);
            }
            else {
                g_Hosts[Ack.host].buffer.front().detect_busy++;
                double time = g_Time + DIFS + BinaryExponentialBackoff(g_Hosts[Ack.host].buffer.front().detect_busy);
                Event wait = Event(Ack.host, g_Time, time, DEPARTURE, g_Hosts[Ack.host].buffer.front().dest, g_Hosts[Ack.host].buffer.front());;
                g_Gel.push(wait);
            }

        }
        else {
            if (g_Hosts[Ack.host].buffer.size() == MAX_BUFFER) {
                g_Packets_Dropped++;
            }
        }
    }
    else {
        g_Packets_Dropped++;
    }

}

/*
 *  Helper function called for every event pulled from the g_System.Gel
 *      Calls the correct process function for an event by type
 */
void ProcessEvent(Event &NextEvent) {
    g_Time = NextEvent.end_time;
    if (NextEvent.type == 'A') { ProcessArrival(NextEvent); }
    else if (NextEvent.type == 'D') { ProcessDeparture(NextEvent); }
    else if (NextEvent.type == 'S') { ProcessSent(NextEvent); }
    else if (NextEvent.type == 'K') { ProcessAck(NextEvent); }
    else if (NextEvent.type == 'T') { ProcessAckTimer(NextEvent); }
}
/*
 *  
 *      
 */
void PostEventUpkeep() {

    //sort vectors by end_time

}

/*
 *  Output formatter for displaying simulation results to terminal
 */
void SimulationOutput() {
    double utilization = (g_Time - g_Idle_Time) / (g_Time);
    SIM_OUT("*======================================*");
    SIM_OUT("|          SIMULATION COMPLETE         |");
    SIM_OUT("*======================================*");
    SIM_OUT("   Lambda: " + to_string(g_Lambda));
    SIM_OUT("   Hosts: " + to_string(g_Num_Hosts));
    SIM_OUT("   Utilization: " + to_string(utilization));
    SIM_OUT("   Packets Dropped: " + to_string(g_Packets_Dropped));
    SIM_OUT("   Average Network Delay: " + to_string((g_Transmission_Time + g_Queue_Time )/(g_Packets_Sent)));
}

/*
 *  Helper function used to initialize the simulation to its starting state
 */
void Init() {
 
    double lambda;
    int hosts;
    SIM_OUT("Enter value for Lambda: :");
    cin >> lambda;
    SIM_OUT("Enter value for Hosts: ");
    cin >> hosts;

    g_Lambda = lambda;
    g_Num_Hosts = hosts;

    for (int i = 0; i < hosts; i++) {
        Host temp;
        g_Hosts.push_back(temp);

        double firstArrivalTime = NegativeExponential(g_Lambda);
        int destination = generateDestination(i, hosts);

        Packet packet = Packet(false, CreateNewPacketSize(), i, destination, firstArrivalTime);

        Event event = Event(i, g_Time, firstArrivalTime, ARRIVAL, destination, packet);

        g_Gel.push(event);
    }

    SIM_OUT("Initialized Default Values...\n");
}

/*
 *  Main function
 */
int main(int argc, char* argv[]) {
    Init();

    for (int i = 0; i < g_total_events; i++) {

        Event next = g_Gel.front();        
        ProcessEvent(next);
        g_Gel.pop();
        PostEventUpkeep();
    }
    SimulationOutput();
    return 0;
}