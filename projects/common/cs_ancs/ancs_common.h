/* ----------------------------------------------------------------------------
 * Copyright (c) 2020-2030 chipsea Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of chipseaelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -------------------------------------------------------------------------- */

/**
 * @file     ancs_common.h
 * @date     22. Nov. 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef _ANCS_COMMON_H_
#define _ANCS_COMMON_H_

/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>

/*********************************************************************
 * TYPEDEFS
 */
/// Error Codes
enum ancs_error_codes {
    ///  The commandID was not recognized by the NP
    ANCS_UNKNOWN_COMMAND    = 0xA0,
    /// The command was improperly formatted
    ANCS_INVALID_COMMAND    = 0xA1,
    ///  One of the parameters (for example, the NotificationUID) does not refer to an existing object on the NP
    ANCS_INVALID_PARAMETER  = 0xA2,
    ///  The action was not performed.
    ANCS_ACTION_FAILED      = 0xA3,
};

///  CategoryID Values
enum ancs_categoryID_values {
    ANCS_CATEGORY_ID_OTHER              = 0,
    ANCS_CATEGORY_ID_INCOMINGCALL       = 1,
    ANCS_CATEGORY_ID_MISSEDCALL         = 2,
    ANCS_CATEGORY_ID_VOICEMAIL          = 3,
    ANCS_CATEGORY_ID_SOCIAL             = 4,
    ANCS_CATEGORY_ID_SCHEDULE           = 5,
    ANCS_CATEGORY_ID_EMAIL              = 6,
    ANCS_CATEGORY_ID_NEWS               = 7,
    ANCS_CATEGORY_ID_HEALTHANDFITNESS   = 8,
    ANCS_CATEGORY_ID_BUSINESSANDFINANCE = 9,
    ANCS_CATEGORY_ID_LOCATION           = 10,
    ANCS_CATEGORY_ID_ENTERTAINMENT      = 11,
    ANCS_CATEGORY_ID_CONNECTEDCALL      = 12,
    ANCS_CATEGORY_ID_RESERVED           = 255,
};

///  EventID Values
enum ancs_eventID_values {
    ANCS_EVENT_ID_NOTIFICATION_ADDED    = 0,
    ANCS_EVENT_ID_NOTIFICATION_MODIFIED = 1,
    ANCS_EVENT_ID_NOTIFICATION_REMOVED  = 2,
    ANCS_EVENT_ID_RESERVED              = 255,
};

/// EventFlags

enum ancs_event_flags {
    ANCS_EVENT_FLAG_SILENT         = (1 << 0),
    ANCS_EVENT_FLAG_IMPORTANT      = (1 << 1),
    ANCS_EVENT_FLAG_EXISTING       = (1 << 2),
    ANCS_EVENT_FLAG_POSITIVEACTION = (1 << 3),
    ANCS_EVENT_FLAG_NEGATIVEACTION = (1 << 4),
    ANCS_EVENT_FLAG_RESERVED       = 0,
};

///  CommandID Values
enum ancs_commandID_values {
    ANCS_COMMAND_ID_GET_NTF_ATTR    = 0,
    ANCS_COMMAND_ID_APP_ATTR        = 1,
    ANCS_COMMAND_ID_PERFORM_NTF_ACT = 2,
    ANCS_COMMAND_ID_RESERVED        = 255,
};

/// NotificationAttributeID Values
enum ancs_notification_attributeID_values {
    ANCS_NTF_ATTR_ID_APP_IDENTIFIER = 0,
    ANCS_NTF_ATTR_ID_TITLE          = 1,//(Needs to be followed by a 2-bytes max length parameter)
    ANCS_NTF_ATTR_ID_SUBTITLE       = 2,//(Needs to be followed by a 2-bytes max length parameter)
    ANCS_NTF_ATTR_ID_MESSAGE        = 3,//(Needs to be followed by a 2-bytes max length parameter)
    ANCS_NTF_ATTR_ID_MESSAGESIZE    = 4,
    ANCS_NTF_ATTR_ID_DATE           = 5,
    ANCS_NTF_ATTR_ID_POS_ACT_LABEL  = 6,
    ANCS_NTF_ATTR_ID_NEG_ACT_LABEL  = 7,
    ANCS_NTF_ATTR_ID_RESERVED       = 255,
};

/// ActionID Values
enum ancs_actionID_values {
    ANCS_ACTION_ID_POSITIVE = 0,
    ANCS_ACTION_ID_NEGATIVE = 1,
    ANCS_ACTION_ID_RESERVED = 255,
};

#endif /* _ANCS_COMMON_H_ */

/**
  @}
*/
