/*
*   OctomapPlanner
*
*   Copyright (C) 2018  ArduPilot
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Author Ayush Gaud <ayush.gaud[at]gmail.com>
*/

#ifndef DEBUG_DEFINITIONS_H
#define DEBUG_DEFINITIONS_H

#define RED "\x1B[31m"
#define GRN "\x1B[32m"
#define YEL "\x1B[33m"
#define BLUE "\x1B[34m"
#define RESET "\x1B[0m"

#define INFO(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << GRN << str \
              << RESET << std::endl; } while(0)
#define WARN(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << YEL << str \
              << RESET << std::endl; } while(0)
#define ERROR(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << RED << str \
              << RESET << std::endl; } while(0)
#define DBG(str) do { std::cout << __FILE__ << " " << __FUNCTION__<< ": Line " << __LINE__ << ": " << BLUE << str \
              << RESET << std::endl; } while(0)

#endif