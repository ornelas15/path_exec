/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2020 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* main.cpp
 *
 * Basic Robot Agent
 * simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <locale.h>

#include <iostream>

using std::cerr;

#include <QLocale>
#include <qapplication.h>
#include "../libRobSock/RobSock.h"
#include "sampapp.h"
#include "robview.h"
#include "lmap.h"


int main( int argc, char** argv )
{
    char host[100]="localhost";
    char rob_name[20]="GUISample";
    int rob_id=1;
    bool map_valid=false;
    LMap lmap;

    printf(" GUISample Robot \n Copyright 2002-2020 Universidade de Aveiro\n");
    fflush(stdout);

    /* processing arguments */
    while (argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        if (strcmp(argv[1], "--host") == 0 || strcmp(argv[1], "-h") == 0)
        {
            strncpy(host, argv[2], 99);
            host[99]='\0';
        }
        else if (strcmp(argv[1], "--robname") == 0 || strcmp(argv[1], "-r") == 0)
        {
            strncpy(rob_name, argv[2], 19);
            rob_name[19]='\0';
        }
        else if (strcmp(argv[1], "--pos") == 0 || strcmp(argv[1], "-p") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
                argc=0; // error message will be printed
        }
        else if (strcmp(argv[1], "--map") == 0 || strcmp(argv[1], "-m") == 0)
        {

	        QXmlInputSource *source;

            QFile srcFile(argv[2]);

            if(!srcFile.exists()) {
                cerr << "Could not open " << argv[2] << "\n";
                return 1;
            }
            if ((source = new QXmlInputSource(&srcFile)) == 0)
	        {
                cerr << "Fail sourcing map file\n";
		        return 1;
	        }

            QXmlSimpleReader xmlParser;

            xmlParser.setContentHandler(&lmap);
            xmlParser.parse(source);

            map_valid = true;

        }
        else
        {
            break; // the while
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: GUISample [--host hostname] [--robname robotname] [--pos posnumber]\n");
        return 1;
    }

    // Create Qt application - Must be before InitRobot
    SampApp app( argc, argv, rob_name );

    if (map_valid) {
        app.setMap(lmap.lmap);
        app.printMap();
    }

    qApp->addLibraryPath("../libRobSock");

    /* Connect Robot to simulator */
    double irSensorAngles[4] = { 0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(rob_name,rob_id,irSensorAngles,host)==-1) {
        printf("%s Failed to connect\n",rob_name);
        exit(1);
    }
    printf("%s Connected\n",rob_name);
    fflush(stdout);

    // Connect event NewMessage to handler act()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &app, SLOT(act()));

    // create robot display widget
    RobView robView(irSensorAngles, rob_name);

    // Connect event NewMessage to handler redrawRobot()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &robView, SLOT(redrawRobot()));

    robView.show();

    setlocale(LC_ALL,"C");
    QLocale::setDefault(QLocale::c());

    // process events
    return app.exec();
}
