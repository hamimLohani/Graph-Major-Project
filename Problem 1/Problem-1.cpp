
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
    double distance;
    int prev;

    Node(pair<double, double> lon_lat)
    {
        this->lon_lat = lon_lat;
    }
    Node()
    {
    }
};

void dijkstra(int src, vector<Node> &nodes)
{
    for (int i = 1; i < nodes.size(); i++)
    {
        nodes[i].distance = infinity;
        nodes[i].prev = -1;
    }

    nodes[src].distance = 0;

    set<pair<double, int>> st;
    for (int i = 1; i < nodes.size(); i++)
        st.insert({nodes[i].distance, i});

    while (st.size())
    {
        auto v_it = st.begin();
        int v = v_it->second;

        st.erase(v_it);

        for (auto &edge : nodes[v].adj)
        {
            int u = edge.first;
            double vu_w = edge.second;
            auto it = st.find({nodes[u].distance, u});
            if (it != st.end() && nodes[u].distance > nodes[v].distance + vu_w)
            {
                nodes[u].distance = nodes[v].distance + vu_w;
                nodes[u].prev = v;
                st.erase(it);
                st.insert({nodes[u].distance, u});
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
    kml << "<name>Shortest Path</name>\n";

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

int main()
{
    vector<Node> nodes(1); // 1-based index
    map<pair<double, double>, int> nodeMap;
    map<pair<int, int>, int> edgesMode; // (Vertex ID, Vertex ID) to Mode No. 1->walk, 2->Car

    ifstream roadmap("../Roadmap-Dhaka.csv");

    if (!(roadmap.is_open()))
    {
        cout << "Cant open the dataset of roadmap" << nl;
        return 0;
    }

    string line;
    while (getline(roadmap, line))
    {
        vector<pair<double, double>> lon_lats;

        stringstream ss(line);
        string roadName;
        getline(ss, roadName, ',');

        string lon, lat;

        while (getline(ss, lon, ','))
        {
            getline(ss, lat, ',');

            lon_lats.push_back({stod(lon), stod(lat)});
        }

        lon_lats.pop_back();

        for (int i = 0; i < lon_lats.size() - 1; i++)
        {
            int u_id = getVertexID(nodeMap, nodes, lon_lats[i]);
            int v_id = getVertexID(nodeMap, nodes, lon_lats[i + 1]);

            double dist = haversine(lon_lats[i], lon_lats[i + 1]);

            nodes[u_id].adj.push_back({v_id, dist});
            nodes[v_id].adj.push_back({u_id, dist});

            edgesMode[{u_id, v_id}] = 2;
            edgesMode[{v_id, u_id}] = 2;
        }
    }
    roadmap.close();

    pair<double, double> src_lonLat, dst_lonLat;
    cin >> src_lonLat.first;
    cin >> src_lonLat.second;
    cin >> dst_lonLat.first;
    cin >> dst_lonLat.second;

    cout << "Source Longitude = " << src_lonLat.first << nl;
    cout << "Source Latitude = " << src_lonLat.second << nl;
    cout << "Destination Longitude = " << dst_lonLat.first << nl;
    cout << "Destination Latitude = " << dst_lonLat.second << nl;

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

        nodes[srcID].adj.push_back({nearestNode, nearestNodeDist});
        nodes[nearestNode].adj.push_back({srcID, nearestNodeDist});

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

        nodes[dstID].adj.push_back({nearestNode, nearestNodeDist});
        nodes[nearestNode].adj.push_back({dstID, nearestNodeDist});

        edgesMode[{dstID, nearestNode}] = 1;
        edgesMode[{nearestNode, dstID}] = 1;
    }
    else
        dstID = nodeMap[dst_lonLat];

    dijkstra(srcID, nodes);

    if (nodes[dstID].distance == infinity)
    {
        cout << "NO path" << nl;
        cout << nl;
        return 0;
    }

    cout << "Shortest Distance = " << nodes[dstID].distance << "(km)" << nl;
    vector<int> path;

    path.push_back(dstID);

    int ID = nodes[dstID].prev;

    while (ID != -1)
    {
        path.push_back(ID);
        ID = nodes[ID].prev;
    }

    reverse(path.begin(), path.end());

    for (int i = 0; i < path.size() - 1; i++)
    {
        cout << fixed << setprecision(6);
        cout << '(' << nodes[path[i]].lon_lat.first << ',' << nodes[path[i]].lon_lat.second << ')';
        cout << "  ->  ";
        cout << '(' << nodes[path[i + 1]].lon_lat.first << ',' << nodes[path[i + 1]].lon_lat.second << ')';

        cout << " ";
        if (edgesMode[{path[i], path[i + 1]}] == 1)
            cout << "( Walk - ";
        else if (edgesMode[{path[i], path[i + 1]}] == 2)
            cout << "( Car - ";

        double dist = haversine(nodes[path[i]].lon_lat, nodes[path[i + 1]].lon_lat);
        cout << dist << " km )";

        cout << nl;
    }

    writeKML("Problem-1.kml", path, nodes);
    cout << "KML written to Problem-1.kml" << nl;

    return 0;
}
