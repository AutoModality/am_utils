/*
 * amoperatorutils.h
 *
 *  Created on: Jun 8, 2015
 *      Author: dan
 */

#ifndef BRAIN_BOX_MSGS_SRC_AMOPERATORUTILS_H_
#define BRAIN_BOX_MSGS_SRC_AMOPERATORUTILS_H_

namespace brain_box {

class AMOperatorUtils {
public:
	static void changemode(int dir);
	static int kbhit (void);
};

} /* namespace brain_box */

#endif /* BRAIN_BOX_MSGS_SRC_AMOPERATORUTILS_H_ */
