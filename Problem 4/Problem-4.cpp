
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <climits>

using namespace std;
#define nl "\n"
#define ll long long
#define infinity INT_MAX

struct Node
{
    pair<double, double> lon_lat;
    vector<pair<int, double>> adj;
    double cost;
    int prev;
    double arrivalTime;
    double waiting;

    Node(pair<double, double> lon_lat)
    {
        this->lon_lat = lon_lat;
    }
    Node()
    {
    }
};

double convertTimeToMinutes(string timeStr)
{
    int hours, minutes;
    char ampm[3];

    if (sscanf(timeStr.c_str(), "%d:%d%2s", &hours, &minutes, ampm) < 3)
    {
        return 0.0;
    }
    string marker = ampm;
    for (char &c : marker)
        c = tolower(c);

    if (marker == "pm" && hours < 12)
    {
        hours += 12;
    }
    else if (marker == "am" && hours == 12)
    {
        hours = 0;
    }

    return static_cast<double>(hours * 60 + minutes);
}

// Function to calculate distance using Haversine formula
double haversine(pair<double, double> lon_lat1, pair<double, double> lon_lat2)
{
    const double R = 6371.0;

    double lon1_rad = lon_lat1.first * M_PI / 180.0;
    double lat1_rad = lon_lat1.second * M_PI / 180.0;
    double lon2_rad = lon_lat2.first * M_PI / 180.0;
    double lat2_rad = lon_lat2.second * M_PI / 180.0;

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance = R * c;
    return distance;
}

void dijkstra(int src, vector<Node> &nodes, map<pair<int, int>, int> &edgesMode, double startingTime)
{
    for (int i = 1; i < nodes.size(); i++)
    {
        nodes[i].cost = infinity;
        nodes[i].arrivalTime = infinity;
        nodes[i].prev = -1;
    }

    nodes[src].cost = 0;
    nodes[src].arrivalTime = startingTime;

    set<pair<double, int>> st;
    for (int i = 1; i < nodes.size(); i++)
        st.insert({nodes[i].cost, i});

    while (st.size())
    {
        auto v_it = st.begin();
        int v = v_it->second;

        st.erase(v_it);

        int prevMode = 0;
        if (nodes[v].prev != -1)
            prevMode = edgesMode[{nodes[v].prev, v}];

        double possible_waiting = 0;
        double at = nodes[v].arrivalTime;
        int at_INT = at;
        if ((at_INT % 15) || (at - at_INT > 0.0))
        {
            double wait_until = at_INT - (at_INT % 15) + 15;
            possible_waiting = wait_until - at;
        }

        if (nodes[v].cost != infinity)
            for (auto &edge : nodes[v].adj)
            {
                int u = edge.first;
                double vu_w = edge.second;
                int mode = edgesMode[{v, u}];
                auto it = st.find({nodes[u].cost, u});

                double speed;                                                   // km Per Hour
                double dist_vu = haversine(nodes[v].lon_lat, nodes[u].lon_lat); // km

                if (mode == 1)
                    speed = 2;
                else
                    speed = 30;

                double waiting = 0;
                if ((mode == 3 || mode == 4 || mode == 5) && mode != prevMode)
                    waiting = possible_waiting;

                double travelTime = (dist_vu / speed) * 60.0;

                // checking if riding on metro or bus for the first time under 6am to 11pm
                bool isOnTime = 1;
                if (mode > 2 && mode != prevMode)
                {
                    isOnTime = 0;

                    if (nodes[v].arrivalTime + waiting >= 360 && nodes[v].arrivalTime + waiting <= 1380)
                        isOnTime = 1;
                }

                if (it != st.end() && isOnTime && nodes[u].cost > nodes[v].cost + vu_w)
                {
                    nodes[u].cost = nodes[v].cost + vu_w;
                    nodes[u].prev = v;

                    nodes[u].arrivalTime = nodes[v].arrivalTime + travelTime + waiting;
                    nodes[u].waiting = waiting;

                    st.erase(it);
                    st.insert({nodes[u].cost, u});
                }
            }
    }
}

int getVertexID(map<pair<double, double>, int> &nodeMap, vector<Node> &nodes, pair<double, double> lon_lat)
{
    if (nodeMap[lon_lat])
        return nodeMap[lon_lat];
    else
    {
        int newID = nodes.size();

        Node newNode(lon_lat);
        nodes.push_back(newNode);

        nodeMap[lon_lat] = newID;

        return newID;
    }
}

void writeKML(
    const string &filename,
    const vector<int> &path,
    const vector<Node> &nodes)
{
    ofstream kml(filename);
    if (!kml.is_open())
    {
        cout << "Could not write KML file" << nl;
        return;
    }

    kml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    kml << "<Document>\n";
    kml << "<name>Cheapest Path with time</name>\n";

    kml << "<Placemark>\n";
    kml << "<name>Route</name>\n";
    kml << "<Style>\n";
    kml << "<LineStyle>\n";
    kml << "<color>ff0000ff</color>\n";
    kml << "<width>4</width>\n";
    kml << "</LineStyle>\n";
    kml << "</Style>\n";

    kml << "<LineString>\n";
    kml << "<tessellate>1</tessellate>\n";
    kml << "<coordinates>\n";

    for (int id : path)
    {
        double lon = nodes[id].lon_lat.first;
        double lat = nodes[id].lon_lat.second;
        kml << lon << "," << lat << ",0\n";
    }

    kml << "</coordinates>\n";
    kml << "</LineString>\n";
    kml << "</Placemark>\n";

    kml << "</Document>\n";
    kml << "</kml>\n";

    kml.close();
}
string trim(const string &s)
{
    const string WHITESPACE = " \n\r\t\f\v";

    size_t start = s.find_first_not_of(WHITESPACE);
    if (start == string::npos)
        return "";

    size_t end = s.find_last_not_of(WHITESPACE);

    return s.substr(start, end - start + 1);
}

void buildGraph_from_dataset(string fileName, vector<Node> &nodes, map<pair<double, double>, int> &nodeMap, map<pair<int, int>, int> &edgesMode, int mode)
{
    ifstream mapFile(fileName);

    if (!(mapFile.is_open()))
    {
        cout << "Cant open the dataset of map - " << fileName << nl;
        return;
    }

    int costPerKM;
    if (mode == 2)
        costPerKM = 20; // car
    else if (mode == 3)
        costPerKM = 5; // metro
    else
        costPerKM = 7;

    string line;

    while (getline(mapFile, line))
    {
        vector<pair<double, double>> lon_lats;

        stringstream ss(line);

        string part;
        vector<string> stringParts;

        while (getline(ss, part, ','))
        {
            stringParts.push_back(trim(part));
        }

        /*
            Lines are like this in the dataset -
            Name,Longitude,Latitude,Name1,Name2
        */

        for (int i = 1; i < stringParts.size() - 3; i += 2)
        {
            lon_lats.push_back({stod(stringParts[i]), stod(stringParts[i + 1])});
        }

        for (int i = 0; i < lon_lats.size() - 1; i++)
        {
            int u_id = getVertexID(nodeMap, nodes, lon_lats[i]);
            int v_id = getVertexID(nodeMap, nodes, lon_lats[i + 1]);

            double cost = haversine(lon_lats[i], lon_lats[i + 1]) * costPerKM;

            nodes[u_id].adj.push_back({v_id, cost});
            nodes[v_id].adj.push_back({u_id, cost});

            edgesMode[{u_id, v_id}] = mode;
            edgesMode[{v_id, u_id}] = mode;
        }
    }
    mapFile.close();
}
string convertMinutesToTime(double totalMinutes)
{
    int total = static_cast<int>(totalMinutes);

    total %= 1440;
    if (total < 0)
        total += 1440;

    int hours = total / 60;
    int minutes = total % 60;

    std::string period = (hours >= 12) ? "pm" : "am";

    int displayHour = hours % 12;
    if (displayHour == 0)
        displayHour = 12;

    char buffer[10];
    std::snprintf(buffer, sizeof(buffer), "%02d:%02d%s", displayHour, minutes, period.c_str());

    return std::string(buffer);
}

int main()
{
    vector<Node> nodes(1); // 1-based index
    map<pair<double, double>, int> nodeMap;
    map<pair<int, int>, int> edgesMode; // (Vertex ID, Vertex ID) to Mode No. 1->walk, 2->Car

    buildGraph_from_dataset("../Roadmap-Dhaka.csv", nodes, nodeMap, edgesMode, 2);
    buildGraph_from_dataset("../Routemap-DhakaMetroRail.csv", nodes, nodeMap, edgesMode, 3);
    buildGraph_from_dataset("../Routemap-UttaraBus.csv", nodes, nodeMap, edgesMode, 4);
    buildGraph_from_dataset("../Routemap-BikolpoBus.csv", nodes, nodeMap, edgesMode, 5);

    pair<double, double> src_lonLat, dst_lonLat;
    string startingTime_str;

    cin >> src_lonLat.first;
    cin >> src_lonLat.second;
    cin >> dst_lonLat.first;
    cin >> dst_lonLat.second;
    cin >> startingTime_str;

    cout << "Source Longitude = " << src_lonLat.first << nl;
    cout << "Source Latitude = " << src_lonLat.second << nl;

    cout << "Destination Longitude = " << dst_lonLat.first << nl;
    cout << "Destination Latitude = " << dst_lonLat.second << nl;

    double startingTime = convertTimeToMinutes(startingTime_str);
    cout << "Starting Time = " << startingTime_str << nl;

    int srcID, dstID;

    // setting src node
    if (!nodeMap[src_lonLat])
    {
        int nearestNode;
        double nearestNodeDist = infinity;

        for (int i = 1; i < nodes.size(); i++)
        {
            double dist = haversine(src_lonLat, nodes[i].lon_lat);
            if (dist < nearestNodeDist)
            {
                nearestNodeDist = dist;
                nearestNode = i;
            }
        }

        srcID = nodes.size();
        nodeMap[src_lonLat] = srcID;

        nodes.push_back(Node(src_lonLat));

        nodes[srcID].adj.push_back({nearestNode, nearestNodeDist * 0});
        nodes[nearestNode].adj.push_back({srcID, nearestNodeDist * 0});

        edgesMode[{srcID, nearestNode}] = 1;
        edgesMode[{nearestNode, srcID}] = 1;
    }
    else
        srcID = nodeMap[src_lonLat];

    // setting dst node
    if (!nodeMap[dst_lonLat])
    {
        int nearestNode;
        double nearestNodeDist = infinity;

        for (int i = 1; i < nodes.size(); i++)
        {
            double dist = haversine(dst_lonLat, nodes[i].lon_lat);
            if (dist < nearestNodeDist && i != srcID)
            {
                nearestNodeDist = dist;
                nearestNode = i;
            }
        }

        dstID = nodes.size();
        nodeMap[dst_lonLat] = dstID;

        nodes.push_back(Node(dst_lonLat));

        nodes[dstID].adj.push_back({nearestNode, nearestNodeDist * 0}); // costPerKm = 0
        nodes[nearestNode].adj.push_back({dstID, nearestNodeDist * 0});

        edgesMode[{dstID, nearestNode}] = 1;
        edgesMode[{nearestNode, dstID}] = 1;
    }
    else
        dstID = nodeMap[dst_lonLat];

    dijkstra(srcID, nodes, edgesMode, startingTime);

    if (nodes[dstID].cost == infinity)
    {
        cout << "NO path" << nl;
        cout << nl;
        return 0;
    }

    cout << fixed << setprecision(6) << nl;
    cout << nl << "Cheapest Cost = " << nodes[dstID].cost << "(Tk)" << nl << nl;
    vector<int> path;

    path.push_back(dstID);

    int ID = nodes[dstID].prev;

    while (ID != -1)
    {
        path.push_back(ID);
        ID = nodes[ID].prev;
    }

    reverse(path.begin(), path.end());

    int prevMode = -1;
    double time = startingTime;

    for (int i = 0; i < path.size() - 1; i++)
    {
        cout << fixed << setprecision(6);
        cout << '(' << nodes[path[i]].lon_lat.first << ',' << nodes[path[i]].lon_lat.second << ')';
        cout << "  ->  ";
        cout << '(' << nodes[path[i + 1]].lon_lat.first << ',' << nodes[path[i + 1]].lon_lat.second << ')';

        int mode = edgesMode[{path[i], path[i + 1]}];
        cout << " ";
        if (mode == 1)
            cout << "( Walk  - ";
        else if (mode == 2)
            cout << "( Car   - ";
        else if (mode == 3)
            cout << "( Metro - ";
        else if (mode == 4)
            cout << "(Uttara Bus - ";
        else
            cout << "(Bikolpo Bus - ";

        int costPerKM;
        if (mode == 1)
            costPerKM = 0;
        else if (mode == 2)
            costPerKM = 20;
        else if (mode == 3)
            costPerKM = 5;
        else
            costPerKM = 7;

        double dist = haversine(nodes[path[i]].lon_lat, nodes[path[i + 1]].lon_lat);

        cout << dist * costPerKM << " TK ) - "
             << convertMinutesToTime(nodes[path[i]].arrivalTime + nodes[path[i + 1]].waiting) << " To " << convertMinutesToTime(nodes[path[i + 1]].arrivalTime);

        cout << nl;

        prevMode = mode;
    }

    writeKML("Problem-4.kml", path, nodes);
    cout << nl << "KML written to Problem-4.kml" << nl;

    return 0;
}
