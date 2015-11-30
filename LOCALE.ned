package locale;

import org.mixim.modules.node.Host80211;
import org.mixim.base.modules.BaseNetwork;
import org.mixim.modules.node.WirelessNode;
import org.mixim.base.modules.BaseNetwork;

network LOCALE extends BaseNetwork
{
    parameters:
        int numNodes; // total number of hosts in the network

    submodules:
        node[numNodes]: WirelessNode {
            parameters:
                @display("ls=,0");
                nicType = default("NicCSMA");
        }
}
