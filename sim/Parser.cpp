#include "Parser.hpp"
#include <iostream>
#include <fstream>

Parser::Parser()
{

}

Parcour* Parser::getParcour(std::string& path)
{

    Parcour *p = new Parcour();
    std::ifstream file;
    file.open(path.c_str());
    if(!file)
    {
	std::cerr << "Could not open parcour file: " << path << std::endl;
	return 0;
    }
    
    double res = 0.1;
    
    std::string line;
    int linecnt = 0;
    int startcnt = 0;
    while(std::getline(file, line))
    {
	int cnt = 0;
	for(std::string::iterator it = line.begin(); it != line.end();it++)
	{
	    switch(*it)
	    {
		case 'R':
		    p->addRightGoal(cnt * res, linecnt * res);
		    break;
		case 'L':
		    p->addLeftGoal(cnt * res, linecnt * res);
		    break;
		case 'w':
		case 'W':
		    p->addWall(cnt * res, linecnt * res);
		    break;
		case 'O':
		    p->addObstacle(cnt * res, linecnt * res);
		    break;
		case 'S':
		    if(startcnt == 0)
			p->addLeftStart(cnt * res, linecnt * res);
		    else
			if(startcnt == 1)
			    p->addRightStart(cnt * res, linecnt * res);
			else
			{
			    std::cout << "Error detected more than two start positions" << std::endl;
			    return 0;
			}
		    startcnt++;
		    break;
		case ' ':
		    //free space
		    break;
		default:
		    std::cerr << "Error, got unecpected character '" << *it << "' at line " << linecnt << ":" << cnt<< std::endl;
		    return 0;
		    break;
	    }
	    
	    cnt++;
	}
	linecnt++;
    }    
    
    return p;
}

std::string getValue(const std::string &line, double &value)
{
        size_t end = line.find_first_of(' ');
        if(end == std::string::npos)
        {
            end = line.size();
        }
        if(end == 0)
        {
            throw std::runtime_error("Unexpected end of line parsing odometry info");
        }
        
        
        
        std::string val = line.substr(0, end);
        value = atof(val.c_str());
        
        std::cout << "Value s '" << val << "' val " << value << std::endl;
        
        if(end == line.size())
            return std::string();
        
        return line.substr(end + 1, line.size());  
}

std::vector< StepInfo > Parser::getStepInfo(std::string& path)
{
    std::vector< StepInfo > ret;
    
    std::ifstream file;
    file.open(path.c_str());
    if(!file)
    {
	std::cerr << "Could not open step info file: " << path << std::endl;
	return ret;
    }
    
    size_t stepCnt = 0;
    
    std::string line;
    int linecnt = 0;
    bool done = false;
    while(!done)
    {
        while(line.empty())
        {
            if(!std::getline(file, line))
            {
                done = true;
                break;
            }
        }

        if(done)
            break;
        
        
        StepInfo info;

        while(line.at(0) == '#')
        {
            if(!std::getline(file, line))
                throw std::runtime_error("Parse error");
        }

        std::cout << "Line " << line << std::endl;
        
        line = getValue(line, info.posChangeVariance[0]);
        std::cout << "Line " << line << std::endl;
        line = getValue(line, info.posChangeVariance[1]);

        std::cout << "Got Pos Variance" << std::endl;
        
        do {
            if(!std::getline(file, line))
                throw std::runtime_error("Parse error");
        }
        while(line.at(0) == '#');

        std::cout << "Line " << line << std::endl;

        line = getValue(line, info.dirChange);
        std::cout << "Line " << line << std::endl;
        line = getValue(line, info.dirChangeVariance);

        std::cout << "Got Angle Variance" << std::endl;
        
        ret.push_back(info);
	
        stepCnt++;

        
        if(!std::getline(file, line))
            break;
        while(line.empty() || line.at(0) == '#' )
        {
            if(!std::getline(file, line))
            {
                done = true;
                break;
            }
        }
    }
    
    std::cout << "Found " << stepCnt << " StepInformations " << std::endl;
    for(int i = 0; i < stepCnt; i++)
    {
        std::cout << "Step " << i << std::endl;
        std::cout << "Pos Variance " << ret[i].posChangeVariance.x() << " " << ret[i].posChangeVariance.y() << std::endl;
        std::cout << "Angle " << ret[i].dirChange << " Variance " << ret[i].dirChangeVariance << std::endl;
        std::cout << std::endl;
    }
    
    
    return ret;
}
