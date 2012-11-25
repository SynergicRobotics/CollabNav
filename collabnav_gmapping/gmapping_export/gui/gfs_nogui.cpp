/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 2.0)" 
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/2.0/
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include <unistd.h>
#include "gsp_thread.h"

//CollabNav
#include <omp.h>
#include <iostream>
using namespace std;

using namespace GMapping;

int  main (int argc, char ** argv){
  cerr << "GMAPPING copyright 2004 by Giorgio Grisetti, Cyrill Stachniss," << endl ;
  cerr << "and Wolfram Burgard. To be published under the CreativeCommons license," << endl ;
  cerr << "see: http://creativecommons.org/licenses/by-nc-sa/2.0/" << endl << endl;


	GridSlamProcessorThread* gsp=  new GridSlamProcessorThread;
	if (gsp->init(argc, argv)){
		cout << "GSP INIT ERROR" << endl;
		return -1;
	}
// 	cout <<"GSP INITIALIZED"<< endl;
	if (gsp->loadFiles()){
		cout <<"GSP READFILE ERROR"<< endl;
		return -2;
	}
// 	cout <<"FILES LOADED"<< endl;
	gsp->setMapUpdateTime(1000000);
	gsp->start();
// 	cout <<"THREAD STARTED"<< endl;
	bool done=false;

//     cout << "  Number of processors available = " << omp_get_num_procs ( ) << "\n";
//     cout << "  Number of threads =              " << omp_get_max_threads ( ) << "\n";

    double time = 0;
	while (!done){
		GridSlamProcessorThread::EventDeque events=gsp->getEvents();
		for (GridSlamProcessorThread::EventDeque::iterator it=events.begin(); it!=events.end(); it++){
			cout << flush;
			GridSlamProcessorThread::DoneEvent* doneEvent=dynamic_cast<GridSlamProcessorThread::DoneEvent*>(*it);
			if (doneEvent){
				done=true;
// 				cout <<"DONE!"<< endl;
				gsp->stop();
			}
			if (*it)
				delete(*it);
		}
	}

//     cout << "Number of processors available = " << omp_get_num_procs ( ) << "\n";
//     cout << "Number of threads =              " << omp_get_max_threads ( ) << "\n";
    cout << gsp->m_matcher.gettotalTime() << " ";
//     cout << "time_threads_" << omp_get_max_threads() << " = [ ";

}
