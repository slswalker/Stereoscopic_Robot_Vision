//
//  Timer.h
//  TestOpenCV
//
//  Created by Sam Walker on 1/23/12.
//  Copyright (c) 2012 Sam Walker. All rights reserved.
//

#include <time.h>


class Timer {
	private:
		unsigned long begTime;
		unsigned long pauseTime;
	public:
		void start() {
			begTime = clock();
		}

		unsigned long elapsedTime() {
			return ((unsigned long) clock() - begTime) / CLOCKS_PER_SEC;
		}

		bool isTimeout(unsigned long seconds) {
			return seconds <= elapsedTime();
		}

		void pause()
		{
			pauseTime = elapsedTime();
		}
		void unPause()
		{
			begTime -= (clock() - pauseTime);
		}
};