/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEART_H
#define _ROBOHEART_H

class RoboHeart
{
    public:
        RoboHeart();
        ~RoboHeart();

        bool begin();
        void beat();
};

#endif