#include "turtlebot3_warehouse/order.h"

Order::Order(std::string include_file_path) : 
    include_file_path_(include_file_path)
{std::cout<<"Creating Order..."<<std::endl;}

Order::Order(std::string include_file_path, unsigned int packageNo, unsigned int pickUpLoc, unsigned int dropOffLoc):
include_file_path_(include_file_path), packageNo_(packageNo), pickUpLoc_(pickUpLoc), dropOffLoc_(dropOffLoc)
{
    std::cout<<"Creating Order..."<<std::endl;
    pickUpCoords_ = getCoordinates(pickUpLoc_);
    dropOffCoords_ = getCoordinates(dropOffLoc_);
}

void Order::setPackageNo(unsigned int packageNo)
{
    packageNo_ = packageNo;
}
void Order::setPickUpLoc(unsigned int pickUpLoc)
{
    pickUpLoc_ = pickUpLoc;
}
void Order::setDropOffLoc(unsigned int dropOffLoc)
{
    dropOffLoc_ = dropOffLoc;
}  

unsigned int Order::getPackageNo(void) const
{
    return packageNo_;
}
double Order::getPickUpLoc(void) const
{
    return pickUpLoc_;
}
double Order::getDropOffLoc(void) const
{
    return dropOffLoc_;
}

std::vector<double> Order::getPickUpCoords(void) const
{
    return pickUpCoords_;
}
std::vector<double> Order::getDropOffCoords(void) const
{
    return dropOffCoords_;
}

std::vector<double> Order::getCoordinates(unsigned int location)
{
    std::vector<double> coords;
    std::ifstream addresses_file(addresses_file_path_);
    std::string myText;
    unsigned int addressState = 0;
    int counter = -1;
    int locationNum = 0;
    std::string locationNum_str;
    double xCoord = 0;
    std::string xCoord_str;
    double yCoord = 0;
    std::string yCoord_str;

    if (addresses_file.is_open())
    {
        // std::cout << "addresses.csv reads:" << std::endl;
        while (addresses_file.good()) { 
            getline(addresses_file, myText);
            std::cout << myText << std::endl;
            addressState = 0;
            
            if (counter >= 0) // skip first line (locationNo,xCoord,yCoord)
            {
                for (auto myChar:myText)
                {
                    switch (addressState)
                    {
                    case 0:
                        // all characters until next comma are the locationNum
                        if (myChar == ',')
                        {
                            locationNum = stod(locationNum_str);
                            addressState++;
                            locationNum_str = "";
                        }
                        else
                        {
                            locationNum_str += myChar;
                        }                        
                        break;
                    case 1:
                        // all characters until next comma are the xCoord
                        if (myChar == ',')
                        {
                            xCoord = stod(xCoord_str);
                            addressState++;
                            xCoord_str = "";
                        }
                        else
                        {
                            xCoord_str += myChar;
                        }
                        break;
                    case 2:
                        // all characters until next comma are the yCoord
                        if (myChar == ',')
                        {
                            yCoord = stod(yCoord_str);
                            addressState = 0;
                            yCoord_str = "";
                        }
                        else
                        {
                           yCoord_str += myChar;
                        }
                        break;
                    default:
                        break;                        
                    }

                }            

                if (location == locationNum)
                {
                    coords.push_back(xCoord);
                    coords.push_back(yCoord);
                    // std::cout<<"locationNum: "<<locationNum<<std::endl;
                    // std::cout<<"xCoord: "<<xCoord<<std::endl;
                    // std::cout<<"yCoord: "<<yCoord<<std::endl;
                    break;
                }
                
                locationNum = 0;
                xCoord = 0;
                yCoord = 0;
            }
            counter++;
            
        }        
    }
    else
    {
        std::cout << "package_orders.csv could not be read" <<std::endl;
    }  


    return coords;
}