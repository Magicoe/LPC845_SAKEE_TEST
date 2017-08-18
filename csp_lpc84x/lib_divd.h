/*
%
% Copyright 2012 NXP Semiconductors,
% 411 E Plumeria Dr San Jose CA USA
% All rights are reserved. Reproduction in whole or in part is prohibited
% without the prior written consent of the copyright owner.
%
*/

/*
+++ IDENTIFICATION

PACKAGE           : 
COMPONENT         : 
INTERFACE         : 
TEMPLATE VERSION  : 3
*/

#ifndef __LIB_DIVD_H
#define __LIB_DIVD_H

typedef struct
{
       uint32_t div;
       uint32_t mod;
} udiv_t;

typedef struct
{
       int32_t div;
       int32_t mod;
} sdiv_t;

typedef	struct _DIVD_API {
    int32_t (*sdiv) (int32_t, int32_t);
    uint32_t (*udiv) (int32_t, int32_t);
    sdiv_t (*sdivmod) (int32_t, int32_t);
    udiv_t (*udivmod) (uint32_t, uint32_t);
}  DIVD_API_T;
  
extern const DIVD_API_T  div_api;  //so application program can access	pointer to
                                    // function table

#endif /* __LIB_DIVD_H */
