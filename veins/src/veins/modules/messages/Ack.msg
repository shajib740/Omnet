//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.modules.messages.DemoSafetyMessage;


namespace veins;


packet Ack extends DemoSafetyMessage {
    // meta info
    string Path;
    int hop;
    int RsuID;
    int DesID;
    int SrcID;
    int Counter;
    bool AckMsg = true;
    simtime_t TimeStamp;
    Coord messageOriginPosition;
    // message info
}