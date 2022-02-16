/***********************  Filename: dp_if.c  *********************************/

/*! \file
     \brief Basic functions for PROFIBUS communication with VPC3+.

*/

/*****************************************************************************/
/* contents:

  - function prototypes
  - data structures
  - internal functions

*/
/*****************************************************************************/
/* include hierarchy */
#include <string.h>
#include "platform.h"

/*---------------------------------------------------------------------------*/
/* version                                                                   */
/*---------------------------------------------------------------------------*/
#define DP_VERSION_MAIN_INTERFACE   ((uint8_t)0x06)
#define DP_VERSION_FUNCTION         ((uint8_t)0x01)
#define DP_VERSION_BUGFIX           ((uint8_t)0x14)

/*---------------------------------------------------------------------------*/
/* function prototypes                                                       */
/*---------------------------------------------------------------------------*/
static DP_ERROR_CODE    VPC3_SetConstants          ( uint8_t bSlaveAddress, uint16_t wIdentNumber );
static DP_ERROR_CODE    VPC3_InitBufferStructure   ( void );
#if DP_SUBSCRIBER
   static DP_ERROR_CODE VPC3_InitSubscriber        ( void );
#endif /* #if DP_SUBSCRIBER */

/*---------------------------------------------------------------------------*/
/* global user data definitions                                              */
/*---------------------------------------------------------------------------*/
#ifdef DP_DEBUG_ENABLE
   sDP_DEBUG_BUFFER_ITEM   asDebugBuffer[ MAX_NR_OF_DEBUG ];
   uint8_t                 bDebugBufferIndex;
   uint8_t                 bDebugBufferOverlapped;

   #if REDUNDANCY
      uint8_t              bOldDebugCode_1;
      uint8_t              bOldDetail_1;
      uint8_t              bOldDebugCode_2;
      uint8_t              bOldDetail_2;
   #endif /* #if REDUNDANCY */
#endif /* #ifdef DP_DEBUG_ENABLE */

/*---------------------------------------------------------------------------*/
/* defines                                                                   */
/*---------------------------------------------------------------------------*/
/* Literals */
#define RBL_PRM   ((uint8_t)0x00)
#define RBL_CFG   ((uint8_t)0x01)
#define RBL_SSA   ((uint8_t)0x02)

/*---------------------------------------------------------------------------*/
/* function: GetFirmwareVersion                                              */
/*---------------------------------------------------------------------------*/
void GetFirmwareVersion( psDP_VERSION pVersion )
{
   pVersion->wComponentsInstalled =   0x0000

                         #if DP_MSAC_C1
                                    | DP_COMP_INSTALLED_MSAC1_IFA

                             #if DP_ALARM
                                    | DP_COMP_INSTALLED_SUB_AL
                                  #if DP_ALARM_OVER_SAP50
                                    | DP_COMP_INSTALLED_SUB_AL_50
                                  #endif /* #if DP_ALARM_OVER_SAP50 */
                             #endif /* #if DP_ALARM */

                             #if DPV1_IM_SUPP
                                    | DP_COMP_INSTALLED_IM
                             #endif /* #if DPV1_IM_SUPP */
                         #endif /* #if DP_MSAC_C1 */

                         #if DP_MSAC_C2
                                    | DP_COMP_INSTALLED_MSAC2_IFA

                             #if DPV1_IM_SUPP
                                    | DP_COMP_INSTALLED_IM
                             #endif /* #if DPV1_IM_SUPP */
                         #endif /* #if DP_MSAC_C2 */

                                        ;

   switch( VPC3_GET_ASIC_TYPE() )
   {
      case AT_VPC3B:
      {
         pVersion->wComponentsInstalled |= DP_COMP_INSTALLED_VPC3_B ;
         break;
      } /* case AT_VPC3B: */

      case AT_VPC3C:
      {
         pVersion->wComponentsInstalled |= DP_COMP_INSTALLED_VPC3_C ;
         break;
      } /* case AT_VPC3C: */

      case AT_MPI12X:
      {
         pVersion->wComponentsInstalled |= DP_COMP_INSTALLED_VPC3_D ;
         break;
      } /* case AT_MPI12X: */

      case AT_VPC3S:
      {
         pVersion->wComponentsInstalled |= DP_COMP_INSTALLED_VPC3_S ;
         break;
      } /* case VPC3S: */

      default:
      {
         /* do nothing */
         break;
      } /* default: */
   } /* switch( VPC3_GET_ASIC_TYPE() */

   pVersion->bMainInterface = DP_VERSION_MAIN_INTERFACE;
   pVersion->bFunction      = DP_VERSION_FUNCTION;
   pVersion->bBugfix        = DP_VERSION_BUGFIX;
} /* void GetFirmwareVersion( psDP_VERSION pVersion ) */

/*---------------------------------------------------------------------------*/
/* function: DpWriteDebugBuffer                                              */
/*---------------------------------------------------------------------------*/
#ifdef DP_DEBUG_ENABLE
void DpWriteDebugBuffer( DEBUG_CODE bDebugCode, uint8_t bDetail1, uint8_t bDetail2 )
{
   #if REDUNDANCY
      if( bDetail1 == RED_CHANNEL_1 )
      {
         if( ( bOldDebugCode_1 != bDebugCode ) || ( bOldDetail_1 != bDetail2 ) )
         {
            bOldDebugCode_1 = bDebugCode;
            bOldDetail_1 = bDetail2;
         } /* if( ( bOldDebugCode_1 != bDebugCode ) || ( bOldDetail_1 != bDetail2 ) ) */
         else
         {
            return;
         } /* else of if( ( bOldDebugCode_1 != bDebugCode ) || ( bOldDetail_1 != bDetail2 ) ) */
      } /* if( bDetail1 == RED_CHANNEL_1 ) */
      else
      {
         if( ( bOldDebugCode_2 != bDebugCode ) || ( bOldDetail_2 != bDetail2 ) )
         {
            bOldDebugCode_2 = bDebugCode;
            bOldDetail_2 = bDetail2;
         } /* if( ( bOldDebugCode_2 != bDebugCode ) || ( bOldDetail_2 != bDetail2 ) ) */
         else
         {
            return;
         } /* else of if( ( bOldDebugCode_2 != bDebugCode ) || ( bOldDetail_2 != bDetail2 ) ) */
      } /* else of if( bDetail1 == RED_CHANNEL_1 ) */
   #endif /* #if REDUNDANCY */

   if( bDebugBufferOverlapped == VPC3_FALSE )
   {
      asDebugBuffer[ bDebugBufferIndex ].bDebugCode = bDebugCode;
      asDebugBuffer[ bDebugBufferIndex ].bDetail1   = bDetail1;
      asDebugBuffer[ bDebugBufferIndex ].bDetail2   = bDetail2;

      if( bDebugBufferIndex == ( MAX_NR_OF_DEBUG - 1 ) )
      {
         bDebugBufferOverlapped = VPC3_TRUE;
         bDebugBufferIndex = 0;
      } /* if( bDebugBufferIndex == ( MAX_NR_OF_DEBUG - 1 ) ) */
      else
      {
         bDebugBufferIndex++;
      } /* else of if( bDebugBufferIndex == ( MAX_NR_OF_DEBUG - 1 ) ) */
   } /* if( bDebugBufferOverlapped == VPC3_FALSE ) */
} /* void DpWriteDebugBuffer( DEBUG_CODE debug_code, uint8_t detail_1, uint8_t detail_2 ) */
#endif /* #ifdef DP_DEBUG_ENABLE */

/*---------------------------------------------------------------------------*/
/* function: DpClearDebugBuffer                                              */
/*---------------------------------------------------------------------------*/
#ifdef DP_DEBUG_ENABLE
void DpClearDebugBuffer( void )
{
uint16_t i;

   bDebugBufferOverlapped = VPC3_FALSE;
   bDebugBufferIndex      = 0x00;

   for( i = 0; i < MAX_NR_OF_DEBUG; i++ )
   {
      asDebugBuffer[i].bDebugCode = 0x00;
      asDebugBuffer[i].bDetail1   = 0x00;
      asDebugBuffer[i].bDetail2   = 0x00;
   } /* for( i = 0; i < MAX_NR_OF_DEBUG; i++ ) */
} /* void DpClearDebugBuffer( void ) */
#endif /* #ifdef DP_DEBUG_ENABLE */

/*---------------------------------------------------------------------------*/
/* function: VPC3_MemoryTest                                                 */
/*---------------------------------------------------------------------------*/
/*!
  \brief Check VPC3 memory.
   This function checks the memory of VPC3+. The starting address is 16hex and
   the end address depends on DP_VPC3_4KB_MODE (DpCfg.h).

  \retval DP_OK - memory OK
  \retval DP_VPC3_ERROR - Memory Error
*/
DP_ERROR_CODE VPC3_MemoryTest( void )
{
#if VPC3_SERIAL_MODE
   VPC3_ADR    wAddress;
#else
   VPC3_UNSIGNED8_PTR  pToVpc3;
#endif /* #if VPC3_SERIAL_MODE */
DP_ERROR_CODE  bError;
uint16_t       wTemp;
uint16_t       j;
uint16_t       i;

   /* neccessary, if 4Kbyte mode enabled */
   VPC3_SET_MODE_REG_2( INIT_VPC3_MODE_REG_2 );

   /*-----------------------------------------------------------------------*/
   /* test and clear vpc3 ram                                               */
   /*-----------------------------------------------------------------------*/
   bError = DP_OK;

   #if VPC3_SERIAL_MODE

      j = 0;
      wAddress = bVpc3RwTsAddr;
      for( i = 0x16; i < ASIC_RAM_LENGTH; )
      {
         Vpc3Write( wAddress++, (uint8_t)( j >> 8 ) );
         Vpc3Write( wAddress++, (uint8_t)( j      ) );

         i+=2;
         j++;
      } /* for( i = 0x16; i < ASIC_RAM_LENGTH; ) */

      j = 0;
      wAddress = bVpc3RwTsAddr;
      for( i = 0x16; i < ASIC_RAM_LENGTH; )
      {
         wTemp = (((uint16_t)Vpc3Read( wAddress++ )) << 8 );
         wTemp +=  (uint16_t)Vpc3Read( wAddress++ );
         if( wTemp != j )
         {
            bError = DP_VPC3_ERROR;
         } /* if( wTemp != j ) */

         i+=2;
         j++;
      } /* for( i = 0x16; i < ASIC_RAM_LENGTH; ) */

   #else

      j = 0;
      pToVpc3 = &pVpc3->bTsAddr VPC3_EXTENSION;
      for( i = 0x16; i < ASIC_RAM_LENGTH; )
      {
         *pToVpc3 = (uint8_t)(j>>8);
         pToVpc3++;
         *pToVpc3 = (uint8_t)j;
         pToVpc3++;

         i+=2;
         j++;
      } /* for( i = 0x16; i < ASIC_RAM_LENGTH; ) */

      j = 0;
      pToVpc3 = &pVpc3->bTsAddr VPC3_EXTENSION;
      for( i = 0x16; i < ASIC_RAM_LENGTH; )
      {
         wTemp = (((uint16_t)*pToVpc3) << 8 );
         pToVpc3++;
         wTemp +=  (uint16_t)*pToVpc3;
         pToVpc3++;
         if( wTemp != j )
         {
            bError = DP_VPC3_ERROR;
         }

         i+=2;
         j++;
      } /* for( i = 0x16; i < ASIC_RAM_LENGTH; ) */

   #endif /* #if VPC3_SERIAL_MODE */

   return bError;
} /* DP_ERROR_CODE VPC3_MemoryTest( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_Initialization                                             */
/*---------------------------------------------------------------------------*/
/*!
  \brief Initializes VPC3+.

  \param[in] bSlaveAddress - PROFIBUS slave address ( range: 1..126 )
  \param[in] wIdentNumber - PROFIBUS ident number
  \param[in] sCfgData - default configuration

  \retval DP_OK - Initialization OK
  \retval DP_NOT_OFFLINE_ERROR - Error VPC3 is not in OFFLINE state
  \retval DP_ADDRESS_ERROR - Error, DP Slave address
  \retval DP_CALCULATE_IO_ERROR - Error with configuration bytes
  \retval DP_DOUT_LEN_ERROR - Error with Dout length
  \retval DP_DIN_LEN_ERROR - Error with Din length
  \retval DP_DIAG_LEN_ERROR - Error with diagnostics length
  \retval DP_PRM_LEN_ERROR - Error with parameter assignment data length
  \retval DP_SSA_LEN_ERROR - Error with address data length
  \retval DP_CFG_LEN_ERROR - Error with configuration data length
  \retval DP_LESS_MEM_ERROR - Error Overall, too much memory used
  \retval DP_LESS_MEM_FDL_ERROR - Error Overall, too much memory used
*/
DP_ERROR_CODE VPC3_Initialization( uint8_t bSlaveAddress, uint16_t wIdentNumber, psCFG psCfgData )
{
DP_ERROR_CODE bError;

   /*-----------------------------------------------------------------------*/
   /* initialize global system structure                                    */
   /*-----------------------------------------------------------------------*/

   #ifdef DP_DEBUG_ENABLE
      DpClearDebugBuffer();

      #if REDUNDANCY
         bOldDebugCode_1 = 0xFF;
         bOldDetail_1 = 0xFF;
         bOldDebugCode_2 = 0xFF;
         bOldDetail_2 = 0xFF;
      #endif /* #if REDUNDANCY */
   #endif /* #ifdef DP_DEBUG_ENABLE */

   /*-------------------------------------------------------------------*/
   /* check VPC3 is in OFFLINE                                          */
   /*-------------------------------------------------------------------*/
   if( !VPC3_GET_OFF_PASS() )
   {
      /* neccessary, if 4Kbyte mode enabled */
      VPC3_SET_MODE_REG_2( INIT_VPC3_MODE_REG_2 );

      /* clear VPC3 */
      #if VPC3_SERIAL_MODE
         Vpc3MemSet_( bVpc3RwTsAddr, 0, (ASIC_RAM_LENGTH-0x16) );
      #else
         Vpc3MemSet_( &pVpc3->bTsAddr VPC3_EXTENSION, 0, (ASIC_RAM_LENGTH-0x16) );
      #endif /* #if VPC3_SERIAL_MODE */

      #if DP_INTERRUPT_MASK_8BIT == 0
         pDpSystem->wPollInterruptMask = SM_INTERRUPT_MASK;
      #endif /* #if DP_INTERRUPT_MASK_8BIT == 0 */

      /*--------------------------------------------------------------*/
      /* set constant values                                          */
      /*--------------------------------------------------------------*/
      bError = VPC3_SetConstants( bSlaveAddress, wIdentNumber );
      if( DP_OK == bError )
      {
         /*-----------------------------------------------------------*/
         /* calculate length of input and output data using cfg-data  */
         /*-----------------------------------------------------------*/
         bError = VPC3_CalculateInpOutpLength( &psCfgData->abData[0], psCfgData->bLength );
         if( DP_OK == bError )
         {
            /*--------------------------------------------------------*/
            /* initialize buffer structure                            */
            /*--------------------------------------------------------*/
            bError = VPC3_InitBufferStructure();
            if ( DP_OK == bError )
            {
               /*-----------------------------------------------------*/
               /* initialize subscriber                               */
               /*-----------------------------------------------------*/
               #if DP_SUBSCRIBER
                   bError = VPC3_InitSubscriber();
               #endif /* #if DP_SUBSCRIBER */

               /*-----------------------------------------------------*/
               /* initialize fdl_interface                            */
               /*-----------------------------------------------------*/
               #if DP_FDL
                  sFdl_Init sFdlInit;

                  if( DP_OK == bError )
                  {
                     memset( &sFdlInit, 0, sizeof( sFdl_Init ) );
                     #if DP_MSAC_C1
                        sFdlInit.eC1SapSupported |= C1_SAP_51;
                        #if DP_ALARM_OVER_SAP50
                           sFdlInit.eC1SapSupported |= C1_SAP_50;
                        #endif /* #if DP_ALARM_OVER_SAP50 */
                     #endif /* #if DP_MSAC_C1 */

                     #if DP_MSAC_C2
                        sFdlInit.bC2Enable = VPC3_TRUE;
                        sFdlInit.bC2_NumberOfSaps = C2_NUM_SAPS;
                     #endif /* #if DP_MSAC_C2 */

                     bError = FDL_InitAcyclicServices( bSlaveAddress, sFdlInit );
                  } /* if( DP_OK == bError ) */
               #endif /* #if DP_FDL */
            } /* if( DP_OK == bError ) */
         } /* if( DP_OK == bError ) */
      } /* if( DP_OK == bError ) */

      if( DP_OK == bError )
      {
         #if VPC3_SERIAL_MODE
            CopyToVpc3_( (VPC3_UNSIGNED8_PTR)(Vpc3AsicAddress+0x16), (MEM_UNSIGNED8_PTR)&pVpc3->bTsAddr VPC3_EXTENSION, 0x2A );
         #endif /* #if VPC3_SERIAL_MODE */

         /*-----------------------------------------------------------*/
         /* set real configuration data                               */
         /*-----------------------------------------------------------*/
         VPC3_SET_READ_CFG_LEN( psCfgData->bLength );      /* set configuration length */
         CopyToVpc3_( VPC3_GET_READ_CFG_BUF_PTR(), &psCfgData->abData[0], psCfgData->bLength );
      } /* if( DP_OK == bError ) */
   } /* if( !VPC3_GET_OFF_PASS() ) */
   else
   {
      bError = DP_NOT_OFFLINE_ERROR;
   } /* else of if( !VPC3_GET_OFF_PASS() ) */

   return bError;
} /* DP_ERROR_CODE VPC3_Initialization( uint8_t bSlaveAddress, uint16_t wIdentNumber, psCFG psCfgData ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_Start                                                      */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set VPC3+ to online mode.
  If the ASIC could be correctly initialized with VPC3_Initialization(), it still has to be started. Between initialization and
  start, the user can still initialize buffers in the ASIC.

  Reaction after this service:
  - VPC3+ generates DX_OUT event, all outputs will be cleared
  - VPC3+ generates BAUDRATE_DETECT event, if master connected
  - master sends FDL-Status.req --> slave answers with FDL-Status.resp ( RTS signal! )
*/
void VPC3_Start( void )
{
   #if DP_MSAC_C2
      MSAC_C2_OpenChannel();
   #endif /* #if DP_MSAC_C2 */

   /*-----------------------------------------------------------------------*/
   /* start vpc3                                                            */
   /*-----------------------------------------------------------------------*/
   VPC3_Start__();

   /* Fetch the first diagnosis buffer */
   pDpSystem->pDiagBuffer = VPC3_GetDiagBufPtr();
} /* void VPC3_Start( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_Stop                                                       */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set VPC3+ to offline mode.
*/
void VPC3_Stop( void )
{
   /*-----------------------------------------------------------------------*/
   /* start vpc3                                                            */
   /*-----------------------------------------------------------------------*/
   VPC3_GoOffline();
   do
   {
      /* wait, for offline */
   }while( VPC3_GET_OFF_PASS() );

   #if DP_MSAC_C2
      MSAC_C2_ResetStateMachine();
   #endif /* #if DP_MSAC_C2 */
} /* void  VPC3_Stop( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_SetConstants                                               */
/*---------------------------------------------------------------------------*/
/*!
  \brief Initializes VPC3+ with all constant values.

  \param[in] bSlaveAddress - PROFIBUS slave address ( range: 1..126 )
  \param[in] wIdentNumber - PROFIBUS ident number

  \retval DP_OK - Initialization OK
  \retval DP_ADDRESS_ERROR - Error, DP Slave address
*/
static DP_ERROR_CODE VPC3_SetConstants( uint8_t bSlaveAddress, uint16_t wIdentNumber )
{
DP_ERROR_CODE bError;

   bError = DP_OK;

   pDpSystem->bDinBufsize  = DIN_BUFSIZE;
   pDpSystem->bDoutBufsize = DOUT_BUFSIZE;
   pDpSystem->bPrmBufsize  = PRM_BUFSIZE;
   pDpSystem->bDiagBufsize = DIAG_BUFSIZE;
   pDpSystem->bCfgBufsize  = CFG_BUFSIZE;
   pDpSystem->bSsaBufsize  = SSA_BUFSIZE;

   pDpSystem->wAsicUserRam = ASIC_USER_RAM;

   /*-----------------------------------------------------------------------*/
   /* initialize  control logic                                             */
   /*-----------------------------------------------------------------------*/
   pVpc3->bIntReqReg_L VPC3_EXTENSION                = 0x00;
   pVpc3->bIntReqReg_H VPC3_EXTENSION                = 0x00;
   pVpc3->sReg.sWrite.bIntAck_L VPC3_EXTENSION       = 0xFF;
   pVpc3->sReg.sWrite.bIntAck_H VPC3_EXTENSION       = 0xFF;
   pVpc3->sCtrlPrm.sWrite.bModeReg1_R VPC3_EXTENSION = 0xFF;

   #if VPC3_SERIAL_MODE
      Vpc3Write( bVpc3RwIntReqReg_L, 0x00 );
      Vpc3Write( bVpc3RwIntReqReg_H, 0x00 );

      Vpc3Write( bVpc3WoIntAck_L,    0xFF );
      Vpc3Write( bVpc3WoIntAck_H,    0xFF );
      Vpc3Write( bVpc3WoModeReg1_R,  0xFF );
   #endif /* #if VPC3_SERIAL_MODE */

   /*-----------------------------------------------------------------------*/
   /* set modes of vpc3                                                     */
   /*-----------------------------------------------------------------------*/
   pVpc3->bModeReg0_H VPC3_EXTENSION = INIT_VPC3_MODE_REG_H;
   pVpc3->bModeReg0_L VPC3_EXTENSION = INIT_VPC3_MODE_REG_L;

   pVpc3->sCtrlPrm.sWrite.bModeReg2 VPC3_EXTENSION = INIT_VPC3_MODE_REG_2;
   pVpc3->sCtrlPrm.sWrite.bModeReg3 VPC3_EXTENSION = INIT_VPC3_MODE_REG_3;

   #if VPC3_SERIAL_MODE
      Vpc3Write( bVpc3RwModeReg0_H, INIT_VPC3_MODE_REG_H );
      Vpc3Write( bVpc3RwModeReg0_L, INIT_VPC3_MODE_REG_L );

      Vpc3Write( bVpc3WoModeReg2, INIT_VPC3_MODE_REG_2 );
      Vpc3Write( bVpc3WoModeReg3, INIT_VPC3_MODE_REG_3 );
   #endif /* #if VPC3_SERIAL_MODE */

   /*-----------------------------------------------------------------------*/
   /* set interrupt triggers                                                */
   /*-----------------------------------------------------------------------*/
   pVpc3->sReg.sWrite.bIntMask_H VPC3_EXTENSION = (uint8_t)(~(INIT_VPC3_IND_H));
   pVpc3->sReg.sWrite.bIntMask_L VPC3_EXTENSION = (uint8_t)(~(INIT_VPC3_IND_L));

   pDpSystem->bIntIndHigh = (uint8_t)(~(INIT_VPC3_IND_H));
   pDpSystem->bIntIndLow  = (uint8_t)(~(INIT_VPC3_IND_L));

   #if VPC3_SERIAL_MODE
      Vpc3Write( bVpc3WoIntMask_H, (uint8_t)(~(INIT_VPC3_IND_H)) );
      Vpc3Write( bVpc3WoIntMask_L, (uint8_t)(~(INIT_VPC3_IND_L)) );
   #endif /* #if VPC3_SERIAL_MODE */

   /*-----------------------------------------------------------------------*/
   /* set time-variables                                                    */
   /*-----------------------------------------------------------------------*/
   pVpc3->sCtrlPrm.sWrite.bWdBaudControlVal VPC3_EXTENSION = 0x10;
   pVpc3->sCtrlPrm.sWrite.bMinTsdrVal VPC3_EXTENSION       = 0x0B;

   #if VPC3_SERIAL_MODE
      Vpc3Write( bVpc3WoWdBaudControlVal, 0x10 );
      Vpc3Write( bVpc3WoMinTsdrVal, 0x0B );
   #endif /* #if VPC3_SERIAL_MODE */

   /*-----------------------------------------------------------------------*/
   /* set variables for synch-mode                                          */
   /*-----------------------------------------------------------------------*/
   #if DP_ISOCHRONOUS_MODE
      pVpc3->sCtrlPrm.sWrite.bSyncPwReg VPC3_EXTENSION = SYNCH_PULSEWIDTH;
      pVpc3->sCtrlPrm.sWrite.bGroupSelectReg VPC3_EXTENSION = 0x80;
      pVpc3->sCtrlPrm.sWrite.bControlCommandReg VPC3_EXTENSION  = 0x00;
      pVpc3->sCtrlPrm.sWrite.bGroupSelectMask VPC3_EXTENSION = 0x00;
      pVpc3->sCtrlPrm.sWrite.bControlCommandMask VPC3_EXTENSION  = 0x00;

      #if VPC3_SERIAL_MODE
         Vpc3Write( bVpc3WoSyncPwReg, SYNCH_PULSEWIDTH );
         Vpc3Write( bVpc3WoGroupSelectReg, 0x80 );
         Vpc3Write( bVpc3WoControlCommandReg, 0x00 );
         Vpc3Write( bVpc3WoGroupSelectMask, 0x00 );
         Vpc3Write( bVpc3WoControlCommandMask, 0x00 );
      #endif /* #if VPC3_SERIAL_MODE */
   #endif /* #if DP_ISOCHRONOUS_MODE */

   /*-----------------------------------------------------------------------*/
   /* set user watchdog (dataexchange telegram counter)                     */
   /*-----------------------------------------------------------------------*/
   pVpc3->abUserWdValue[1] VPC3_EXTENSION = (uint8_t)(USER_WD >> 8);
   pVpc3->abUserWdValue[0] VPC3_EXTENSION = (uint8_t)(USER_WD);

   /*-----------------------------------------------------------------------*/
   /* set pointer to FF                                                     */
   /*-----------------------------------------------------------------------*/
   pVpc3->bFdlSapListPtr   VPC3_EXTENSION = VPC3_SAP_CTRL_LIST_START;
   VPC3_SET_EMPTY_SAP_LIST();

   /*-----------------------------------------------------------------------*/
   /* ssa support                                                           */
   /*-----------------------------------------------------------------------*/
   pVpc3->bRealNoAddChange VPC3_EXTENSION = ( SSA_BUFSIZE == 0 ) ? 0xFF : 0x00;

   /*-----------------------------------------------------------------------*/
   /* set profibus ident number                                             */
   /*-----------------------------------------------------------------------*/
   pVpc3->bIdentHigh VPC3_EXTENSION = (uint8_t)(wIdentNumber >> 8);
   pVpc3->bIdentLow  VPC3_EXTENSION = (uint8_t)(wIdentNumber);

   /*-----------------------------------------------------------------------*/
   /* set and check slave address                                           */
   /*-----------------------------------------------------------------------*/
   if( bSlaveAddress < 127 && bSlaveAddress != 0 )
   {
      pVpc3->bTsAddr VPC3_EXTENSION = bSlaveAddress;
   } /* if( bSlaveAddress < 127 && bSlaveAddress != 0 ) */
   else
   {
      bError = DP_ADDRESS_ERROR;
   } /* else of if( bSlaveAddress < 127 && bSlaveAddress != 0 ) */

   return bError;
} /* static DP_ERROR_CODE VPC3_SetConstants( uint8_t bSlaveAddress, uint16_t wIdentNumber ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_InitSubscriber                                             */
/*---------------------------------------------------------------------------*/
#if DP_SUBSCRIBER
static DP_ERROR_CODE VPC3_InitSubscriber( void )
{
DP_ERROR_CODE       bError;
VPC3_UNSIGNED8_PTR  pToVpc3;              /**< pointer to VPC3, µController formatted */
uint16_t            wDxbBufferLength;     /**< segmented length of DxB-Buffer */
uint8_t             bVpc3SegmentAddress;  /**< segment address in VPC3-ASIC */

   bError = DP_OK;

   /*-----------------------------------------------------------------------*/
   /* check buffer length                                                   */
   /*-----------------------------------------------------------------------*/
   if( DP_MAX_DATA_PER_LINK > 244 )
   {
      bError = SSC_MAX_DATA_PER_LINK;
   } /* if( DP_MAX_DATA_PER_LINK > 244 ) */
   else
   {
      #if VPC3_SERIAL_MODE
         /* pointer mc formatted */
         pToVpc3 = (VPC3_UNSIGNED8_PTR)( Vpc3AsicAddress + DP_ORG_LENGTH + SAP_LENGTH + pDpSystem->wVpc3UsedDPV0BufferMemory );
         /* pointer vpc3 formatted */
         bVpc3SegmentAddress = (uint8_t)( ( pDpSystem->wVpc3UsedDPV0BufferMemory + DP_ORG_LENGTH + SAP_LENGTH ) >> SEG_MULDIV );
      #else
         /* pointer mc formatted */
         pToVpc3 = &pVpc3->abDpBuffer[pDpSystem->wVpc3UsedDPV0BufferMemory] VPC3_EXTENSION;
         /* pointer vpc3 formatted */
         bVpc3SegmentAddress = (uint8_t)( ((VPC3_ADR)pToVpc3-(VPC3_ADR)Vpc3AsicAddress) >> SEG_MULDIV );
      #endif /* #if VPC3_SERIAL_MODE */

      /* length dxb_out */
      wDxbBufferLength = (((DP_MAX_DATA_PER_LINK+2)+SEG_OFFSET) & SEG_ADDBYTE);
      pVpc3->bLenDxbOutBuf = (DP_MAX_DATA_PER_LINK+2);
      pDpSystem->wVpc3UsedDPV0BufferMemory += (3*wDxbBufferLength);
      /* length link status */
      pVpc3->bLenDxbStatusBuf = ((((DP_MAX_LINK_SUPPORTED*2)+4)+SEG_OFFSET) & SEG_ADDBYTE);
      pDpSystem->wVpc3UsedDPV0BufferMemory += pVpc3->bLenDxbStatusBuf;
      /* length link table */
      pVpc3->bLenDxbLinkBuf = (((DP_MAX_LINK_SUPPORTED*4)+SEG_OFFSET) & SEG_ADDBYTE);
      pDpSystem->wVpc3UsedDPV0BufferMemory += pVpc3->bLenDxbLinkBuf;

      /*-------------------------------------------------------------------*/
      /* check memory consumption                                          */
      /*-------------------------------------------------------------------*/
      if( pDpSystem->wVpc3UsedDPV0BufferMemory > ASIC_USER_RAM )
      {
         /* Error: user needs too much memory */
         pDpSystem->wVpc3UsedDPV0BufferMemory = 0;
         bError = DP_LESS_MEM_ERROR;
      } /* if( pDpSystem->wVpc3UsedDPV0BufferMemory > ASIC_USER_RAM ) */
      else
      {
         /*---------------------------------------------------------------*/
         /* set buffer pointer                                            */
         /*---------------------------------------------------------------*/
         pVpc3->bDxbLinkBufPtr   = bVpc3SegmentAddress;
         pVpc3->bDxbStatusBufPtr = pVpc3->bDxbLinkBufPtr   + ( pVpc3->bLenDxbLinkBuf >> SEG_MULDIV );
         pVpc3->bDxbOutBufPtr1   = pVpc3->bDxbStatusBufPtr + ( pVpc3->bLenDxbStatusBuf >> SEG_MULDIV );
         pVpc3->bDxbOutBufPtr2   = pVpc3->bDxbOutBufPtr1   + ( wDxbBufferLength >> SEG_MULDIV );
         pVpc3->bDxbOutBufPtr3   = pVpc3->bDxbOutBufPtr2   + ( wDxbBufferLength >> SEG_MULDIV );

         pDpSystem->pDxbOutBuffer1 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->bDxbOutBufPtr1 VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDxbOutBuffer2 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->bDxbOutBufPtr2 VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDxbOutBuffer3 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->bDxbOutBufPtr3 VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
      } /* else of if( pDpSystem->wVpc3UsedDPV0BufferMemory > ASIC_USER_RAM ) */
   } /* else of if( DP_MAX_DATA_PER_LINK > 244 ) */

   return bError;
} /* static DP_ERROR_CODE VPC3_InitSubscriber( void ) */
#endif /* #if DP_SUBSCRIBER */


/*---------------------------------------------------------------------------*/
/* function: VPC3_CalculateInpOutpLength                                     */
/*---------------------------------------------------------------------------*/
/*!
   \brief Calculation of input- and output data.

   \param[in] pToCfgData - address of configuration data
   \param[in] bCfgLength - length of configuration data

   \retval DP_OK - Initialization OK
   \retval DP_CFG_LEN_ERROR - Error with CFG length
   \retval DP_CALCULATE_IO_ERROR - Error with DIN or DOUT length
   \retval DP_CFG_FORMAT_ERROR - Error in special configuration format
*/
DP_ERROR_CODE VPC3_CalculateInpOutpLength( MEM_UNSIGNED8_PTR pToCfgData, uint8_t bCfgLength )
{
DP_ERROR_CODE bError;
uint8_t       bSpecificDataLength;
uint8_t       bTempOutputDataLength;
uint8_t       bTempInputDataLength;
uint8_t       bLength;
uint8_t       bCount;

   bError = DP_OK;
   bTempOutputDataLength = 0;
   bTempInputDataLength  = 0;

   if( ( bCfgLength > 0 ) && ( bCfgLength <= CFG_BUFSIZE ) )
   {
      for( ; bCfgLength > 0; bCfgLength -= bCount )
      {
         bCount = 0;

         if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT )
         {
            /* general identifier format */
            bCount++;
            /* pToCfgData points to "Configurationbyte", CFG_BF means "CFG_IS_BYTE_FORMAT" */
            bLength = (uint8_t)( ( *pToCfgData & VPC3_CFG_BF_LENGTH) + 1 );

            if( *pToCfgData & VPC3_CFG_BF_OUTP_EXIST )
            {
               bTempOutputDataLength += ( *pToCfgData & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_BF_OUTP_EXIST ) */

            if( *pToCfgData & VPC3_CFG_BF_INP_EXIST )
            {
               bTempInputDataLength += ( *pToCfgData & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_BF_INP_EXIST ) */

            pToCfgData++;
         } /* if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT ) */
         else
         {
            /* pToCfgData points to the headerbyte of "special identifier format */
            /* CFG_SF means "CFG_IS_SPECIAL_FORMAT" */
            if( *pToCfgData & VPC3_CFG_SF_OUTP_EXIST )
            {
               bCount++;    /* next byte contains the length of outp_data */
               bLength = (uint8_t)( ( *( pToCfgData + bCount ) & VPC3_CFG_SF_LENGTH ) + 1 );

               bTempOutputDataLength += ( *( pToCfgData + bCount ) & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_SF_OUTP_EXIST ) */

            if( *pToCfgData & VPC3_CFG_SF_INP_EXIST )
            {
               bCount++;  /* next byte contains the length of inp_data */
               bLength = (uint8_t)( ( *( pToCfgData + bCount ) & VPC3_CFG_SF_LENGTH ) + 1 );

               bTempInputDataLength += ( *( pToCfgData + bCount ) & VPC3_CFG_LENGTH_IS_WORD_FORMAT ) ? ( 2 * bLength ) : bLength;
            } /* if( *pToCfgData & VPC3_CFG_SF_INP_EXIST ) */

            bSpecificDataLength = (uint8_t)( *pToCfgData & VPC3_CFG_BF_LENGTH );

            if( bSpecificDataLength != 15 )
            {
               bCount = (uint8_t)( bCount + 1 + bSpecificDataLength );
               pToCfgData = pToCfgData + bCount;
            } /* if( bSpecificDataLength != 15 ) */
            else
            {
               /* specific data length = 15 not allowed */
               pDpSystem->bInputDataLength  = 0xFF;
               pDpSystem->bOutputDataLength = 0xFF;
               bError = DP_CFG_FORMAT_ERROR;
            } /* else of if( bSpecificDataLength != 15 ) */
         } /* else of if( *pToCfgData & VPC3_CFG_IS_BYTE_FORMAT ) */
      } /* for( ; bCfgLength > 0; bCfgLength -= bCount ) */

      if( ( bCfgLength != 0 ) || ( bTempInputDataLength > DIN_BUFSIZE ) || ( bTempOutputDataLength > DOUT_BUFSIZE ) )
      {
         pDpSystem->bInputDataLength  = 0xFF;
         pDpSystem->bOutputDataLength = 0xFF;

         bError = DP_CALCULATE_IO_ERROR;
      } /* if( ( bCfgLength != 0 ) || ( bTempInputDataLength > DIN_BUFSIZE ) || ( bTempOutputDataLength > DOUT_BUFSIZE ) ) */
      else
      {
         pDpSystem->bInputDataLength  = bTempInputDataLength;
         pDpSystem->bOutputDataLength = bTempOutputDataLength;
         bError = DP_OK;
      } /* else of if( ( bCfgLength != 0 ) || ( bTempInputDataLength > DIN_BUFSIZE ) || ( bTempOutputDataLength > DOUT_BUFSIZE ) ) */
   } /* if( ( bCfgLength > 0 ) && ( bCfgLength <= CFG_BUFSIZE ) ) */
   else
   {
      pDpSystem->bInputDataLength  = 0xFF;
      pDpSystem->bOutputDataLength = 0xFF;
      bError = DP_CFG_LEN_ERROR;
   } /* else of if( ( bCfgLength > 0 ) && ( bCfgLength <= CFG_BUFSIZE ) ) */

   return bError;
} /* DP_ERROR_CODE VPC3_CalculateInpOutpLength( MEM_UNSIGNED8_PTR pToCfgData, uint8_t bCfgLength ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_InitBufferStructure                                        */
/*---------------------------------------------------------------------------*/
/*!
  \brief Initializes VPC3+ buffer structure.

  \retval DP_OK - Initialization OK
  \retval DP_DOUT_LEN_ERROR - Error with Dout length
  \retval DP_DIN_LEN_ERROR - Error with Din length
  \retval DP_DIAG_LEN_ERROR - Error with diagnostics length
  \retval DP_PRM_LEN_ERROR - Error with parameter assignment data length
  \retval DP_SSA_LEN_ERROR - Error with address data length
  \retval DP_CFG_LEN_ERROR - Error with configuration data length
  \retval DP_LESS_MEM_ERROR - Error Overall, too much memory used
*/
static DP_ERROR_CODE VPC3_InitBufferStructure( void )
{
DP_ERROR_CODE  bError;
uint16_t       wOutBufferLength;    /**< calculated length of output buffer */
uint16_t       wInBufferLength;     /**< calculated length of input buffer */
uint16_t       wDiagBufferLength;   /**< calculated length of diagnostic buffer */
uint16_t       wPrmBufferLength;    /**< calculated length of parameter buffer */
uint16_t       wCfgBufferLength;    /**< calculated length of check-config buffer */
uint16_t       wSsaBufferLength;    /**< calculated length of set-slave-address buffer */
uint16_t       wAux2BufferLength;   /**< calculated length of aux buffer 2*/

   bError = DP_OK;

   /*-----------------------------------------------------------------------*/
   /* check buffer length                                                   */
   /*-----------------------------------------------------------------------*/
   if( pDpSystem->bDoutBufsize > 244 )
   {
      bError = DP_DOUT_LEN_ERROR;
   } /* if( pDpSystem->bDoutBufsize > 244 ) */

   else
   if( pDpSystem->bDinBufsize > 244 )
   {
      bError = DP_DIN_LEN_ERROR;
   } /* if( pDpSystem->bDinBufsize > 244 ) */

   else
   if( ( pDpSystem->bDiagBufsize < 6 ) || ( pDpSystem->bDiagBufsize > 244 ) )
   {
      bError = DP_DIAG_LEN_ERROR;
   } /* if( ( pDpSystem->bDiagBufsize < 6 ) || ( pDpSystem->bDiagBufsize > 244 ) */

   else
   if( ( pDpSystem->bPrmBufsize < 7 ) || ( pDpSystem->bPrmBufsize > 244 ) )
   {
      bError = DP_PRM_LEN_ERROR;
   } /* if( ( pDpSystem->bPrmBufsize < 7 ) || ( pDpSystem->bPrmBufsize > 244 ) ) */

   else
   if( ( pDpSystem->bCfgBufsize < 1 ) || ( pDpSystem->bCfgBufsize > 244 ) )
   {
      bError = DP_CFG_LEN_ERROR;
   } /* if( ( pDpSystem->bCfgBufsize < 1 ) || ( pDpSystem->bCfgBufsize > 244 ) ) */

   else
   if( pDpSystem->bSsaBufsize != 0 && ( ( pDpSystem->bSsaBufsize < 4 ) || ( pDpSystem->bSsaBufsize > 244 ) ) )
   {
      bError = DP_SSA_LEN_ERROR;
   } /* if( pDpSystem->bSsaBufsize != 0 && ( ( pDpSystem->bSsaBufsize < 4 ) || ( pDpSystem->bSsaBufsize > 244 ) ) ) */

   /* prm buffer */
   if( pVpc3->bModeReg0_H VPC3_EXTENSION & VPC3_SPEC_PRM_BUF_MODE )
   {
      /* Spec_Prm_Buf_Mode: Is no longer supported by the software, 4kByte memory is enough to handle Set-Prm-telegram via auxiliary buffer. */
      bError = DP_SPEC_PRM_NOT_SUPP_ERROR;
   } /* if( pVpc3->bModeReg0_H VPC3_EXTENSION & VPC3_SPEC_PRM_BUF_MODE ) */
   pVpc3->bLenSpecPrmBuf VPC3_EXTENSION = 0;

   if( bError == DP_OK )
   {
      /*-------------------------------------------------------------------*/
      /* length of buffers ok, check memory consumption                    */
      /*-------------------------------------------------------------------*/
      /* length of output buffer */
      wOutBufferLength =  ( ( pDpSystem->bDoutBufsize + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of input buffer */
      wInBufferLength =   ( ( pDpSystem->bDinBufsize  + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of diagnostic buffer */
      wDiagBufferLength = ( ( pDpSystem->bDiagBufsize + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of prm buffer */
      wPrmBufferLength =  ( ( pDpSystem->bPrmBufsize  + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of cfg buffer */
      wCfgBufferLength =  ( ( pDpSystem->bCfgBufsize  + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of ssa buffer */
      wSsaBufferLength =  ( ( pDpSystem->bSsaBufsize  + SEG_OFFSET ) & SEG_ADDWORD );
      /* length of aux buffer 2 2*/
      wAux2BufferLength = ( wCfgBufferLength > wSsaBufferLength ) ? wCfgBufferLength : wSsaBufferLength;

      /*-------------------------------------------------------------------*/
      /* check memory consumption                                          */
      /*-------------------------------------------------------------------*/
      pDpSystem->wVpc3UsedDPV0BufferMemory = 0;
      /* add input and output buffer */
      pDpSystem->wVpc3UsedDPV0BufferMemory += ( wOutBufferLength + wInBufferLength ) * 3;
      /* add diagnostic buffer */
      pDpSystem->wVpc3UsedDPV0BufferMemory += wDiagBufferLength * 2;
      /* add prm buffer, add aux buffer 1 */
      pDpSystem->wVpc3UsedDPV0BufferMemory += wPrmBufferLength * 2;
      /* add Read Config fuffer, add Check Config buffer */
      pDpSystem->wVpc3UsedDPV0BufferMemory += wCfgBufferLength * 2;
      /* add SSA buffer */
      pDpSystem->wVpc3UsedDPV0BufferMemory += wSsaBufferLength;
      /* add aux buffer 2 */
      pDpSystem->wVpc3UsedDPV0BufferMemory += wAux2BufferLength;

      if( pDpSystem->wVpc3UsedDPV0BufferMemory > pDpSystem->wAsicUserRam )
      {
         /* Error: user needs too much memory */
         pDpSystem->wVpc3UsedDPV0BufferMemory = 0;
         bError = DP_LESS_MEM_ERROR;
      } /* if( pDpSystem->wVpc3UsedDPV0BufferMemory > pDpSystem->wAsicUserRam ) */
      else
      {
         /*---------------------------------------------------------------*/
         /* set buffer pointer                                            */
         /*---------------------------------------------------------------*/
         pVpc3->abDoutBufPtr[0] VPC3_EXTENSION  = VPC3_DP_BUF_START;
         pVpc3->abDoutBufPtr[1] VPC3_EXTENSION  = pVpc3->abDoutBufPtr[0] VPC3_EXTENSION  + ( wOutBufferLength  >> SEG_MULDIV );
         pVpc3->abDoutBufPtr[2] VPC3_EXTENSION  = pVpc3->abDoutBufPtr[1] VPC3_EXTENSION  + ( wOutBufferLength  >> SEG_MULDIV );

         pVpc3->abDinBufPtr[0] VPC3_EXTENSION   = pVpc3->abDoutBufPtr[2] VPC3_EXTENSION  + ( wOutBufferLength  >> SEG_MULDIV );
         pVpc3->abDinBufPtr[1] VPC3_EXTENSION   = pVpc3->abDinBufPtr[0]  VPC3_EXTENSION  + ( wInBufferLength   >> SEG_MULDIV );
         pVpc3->abDinBufPtr[2] VPC3_EXTENSION   = pVpc3->abDinBufPtr[1]  VPC3_EXTENSION  + ( wInBufferLength   >> SEG_MULDIV );

         pVpc3->abDiagBufPtr[0] VPC3_EXTENSION  = pVpc3->abDinBufPtr[2]  VPC3_EXTENSION  + ( wInBufferLength   >> SEG_MULDIV );
         pVpc3->abDiagBufPtr[1] VPC3_EXTENSION  = pVpc3->abDiagBufPtr[0] VPC3_EXTENSION  + ( wDiagBufferLength >> SEG_MULDIV );

         pVpc3->bCfgBufPtr VPC3_EXTENSION       = pVpc3->abDiagBufPtr[1] VPC3_EXTENSION  + ( wDiagBufferLength >> SEG_MULDIV );
         pVpc3->bReadCfgBufPtr VPC3_EXTENSION   = pVpc3->bCfgBufPtr      VPC3_EXTENSION  + ( wCfgBufferLength  >> SEG_MULDIV );

         pVpc3->bPrmBufPtr VPC3_EXTENSION       = pVpc3->bReadCfgBufPtr  VPC3_EXTENSION  + ( wCfgBufferLength  >> SEG_MULDIV );

         pVpc3->bAuxBufSel VPC3_EXTENSION       = 0x06; /* SetPrm via Aux1, ChkCfg and SSA via Aux2 */
         pVpc3->abAuxBufPtr[0] VPC3_EXTENSION   = pVpc3->bPrmBufPtr VPC3_EXTENSION       + ( wPrmBufferLength  >> SEG_MULDIV );
         pVpc3->abAuxBufPtr[1] VPC3_EXTENSION   = pVpc3->abAuxBufPtr[0] VPC3_EXTENSION   + ( wPrmBufferLength  >> SEG_MULDIV );

         pVpc3->bSsaBufPtr VPC3_EXTENSION = 0;
         if( wSsaBufferLength != 0 )
         {
             pVpc3->bSsaBufPtr VPC3_EXTENSION   = pVpc3->abAuxBufPtr[1] VPC3_EXTENSION   + ( wAux2BufferLength >> SEG_MULDIV );
         } /* if( wSsaBufferLength != 0 ) */

         /*---------------------------------------------------------------*/
         /* set buffer length                                             */
         /*---------------------------------------------------------------*/
         pVpc3->bLenDoutBuf VPC3_EXTENSION     = pDpSystem->bOutputDataLength;
         pVpc3->bLenDinBuf  VPC3_EXTENSION     = pDpSystem->bInputDataLength;

         pVpc3->abLenDiagBuf[0] VPC3_EXTENSION = 6;
         pVpc3->abLenDiagBuf[1] VPC3_EXTENSION = 6;

         pVpc3->bLenCfgData VPC3_EXTENSION     = pDpSystem->bCfgBufsize;
         pVpc3->bLenPrmData VPC3_EXTENSION     = pDpSystem->bPrmBufsize;
         pVpc3->bLenSsaBuf  VPC3_EXTENSION     = pDpSystem->bSsaBufsize;

         pVpc3->abLenCtrlBuf[0] VPC3_EXTENSION = pDpSystem->bPrmBufsize;
         pVpc3->abLenCtrlBuf[1] VPC3_EXTENSION = ( wAux2BufferLength >= 244 ) ? 244 : wAux2BufferLength;

         /* for faster access, store some pointer in local structure */
         pDpSystem->pDoutBuffer1 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDoutBufPtr[0] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));                                                                                                                                                                                                       ;
         pDpSystem->pDoutBuffer2 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDoutBufPtr[1] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDoutBuffer3 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDoutBufPtr[2] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));

         pDpSystem->pDinBuffer1  = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDinBufPtr[0] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDinBuffer2  = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDinBufPtr[1] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDinBuffer3  = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDinBufPtr[2] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));

         pDpSystem->pDiagBuffer1 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDiagBufPtr[0] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
         pDpSystem->pDiagBuffer2 = (VPC3_UNSIGNED8_PTR)(((VPC3_ADR)(pVpc3->abDiagBufPtr[1] VPC3_EXTENSION) << SEG_MULDIV)+((VPC3_ADR)Vpc3AsicAddress));
      } /* else of if( pDpSystem->wVpc3UsedDPV0BufferMemory > pDpSystem->wAsicUserRam ) */
   } /* if( bError == DP_OK ) */

   return bError;
} /* static DP_ERROR_CODE VPC3_InitBufferStructure( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_GetDoutBufPtr                                              */
/*---------------------------------------------------------------------------*/
/*!
  \brief Fetch buffer pointer of the current output buffer.

  \param[out] pbState

  \retval !0 - pointer to current output buffer.
  \retval 0 - no buffer available
*/
VPC3_UNSIGNED8_PTR VPC3_GetDoutBufPtr( MEM_UNSIGNED8_PTR pbState )
{
VPC3_UNSIGNED8_PTR pToOutputBuffer;             /* pointer to output buffer ( DP-Master -> VPC3+ ) */

   *pbState = VPC3_GET_NEXT_DOUT_BUFFER_CMD();

   switch( VPC3_GET_DOUT_BUFFER_SM() )          /* locate user Dout Buffer */
   {
      case 0x10:
      {
         pToOutputBuffer = pDpSystem->pDoutBuffer1;
         break;
      } /* case 0x10: */

      case 0x20:
      {
         pToOutputBuffer = pDpSystem->pDoutBuffer2;
         break;
      } /* case 0x20: */

      case 0x30:
      {
         pToOutputBuffer = pDpSystem->pDoutBuffer3;
         break;
      } /* case 0x30: */

      default:
      {
         pToOutputBuffer = VPC3_NULL_PTR;
         break;
      } /* default: */
   } /* switch( VPC3_GET_DOUT_BUFFER_SM() ) */

   return pToOutputBuffer;
} /* VPC3_UNSIGNED8_PTR VPC3_GetDoutBufPtr( MEM_UNSIGNED8_PTR pbState ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_GetDinBufPtr                                               */
/*---------------------------------------------------------------------------*/
/*!
  \brief Fetch buffer pointer of the next input buffer.

  \retval !0 - pointer to current input buffer.
  \retval 0 - no buffer available
*/
VPC3_UNSIGNED8_PTR VPC3_GetDinBufPtr( void )
{
VPC3_UNSIGNED8_PTR pToInputBuffer;     /* pointer to input buffer ( VPC3 -> DP-Master ) */

   switch( VPC3_GET_DIN_BUFFER_SM() )  /* locate user Din Buffer */
   {
      case 0x10:
      {
         pToInputBuffer = pDpSystem->pDinBuffer1;
         break;
      } /* case 0x10: */

      case 0x20:
      {
         pToInputBuffer = pDpSystem->pDinBuffer2;
         break;
      } /* case 0x20: */

      case 0x30:
      {
         pToInputBuffer = pDpSystem->pDinBuffer3;
         break;
      } /* case 0x30: */

      default:
      {
         pToInputBuffer = VPC3_NULL_PTR;
         break;
      } /* default: */
   } /* switch( VPC3_GET_DIN_BUFFER_SM() ) */

   return pToInputBuffer;
} /* VPC3_UNSIGNED8_PTR VPC3_GetDinBufPtr( void ) */

/*--------------------------------------------------------------------------*/
/* function: VPC3_InputDataUpdate                                           */
/*--------------------------------------------------------------------------*/

/*!
  \brief Copy input data to VPC3+ and perform buffer exchange.

  \param[in] pToInputData - pointer to local input data
*/
void VPC3_InputDataUpdate( MEM_UNSIGNED8_PTR pToInputData )
{
VPC3_UNSIGNED8_PTR   pToInputBuffer;
volatile uint8_t     bResult;

   /* write DIN data to VPC3 */
   pToInputBuffer = VPC3_GetDinBufPtr();
   if( pToInputBuffer != VPC3_NULL_PTR )
   {
      CopyToVpc3_( pToInputBuffer, pToInputData, (uint16_t)pDpSystem->bInputDataLength );

      /* the user makes a new Din-Buffer available in the state N */
      bResult = VPC3_INPUT_UPDATE();
   } /* if( pToInputBuffer != VPC3_NULL_PTR ) */
} /* void VPC3_InputDataUpdate( MEM_UNSIGNED8_PTR pToInputData ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_GetDxbBufPtr                                               */
/*---------------------------------------------------------------------------*/
/*!
  \brief Fetch buffer pointer of the current dxb buffer.

  \param[out] pbState

  \retval !0 - pointer to current dxb buffer.
  \retval 0 - no buffer available
*/
#if DP_SUBSCRIBER
VPC3_UNSIGNED8_PTR VPC3_GetDxbBufPtr( MEM_UNSIGNED8_PTR pbState )
{
VPC3_UNSIGNED8_PTR pToDxbBuffer;

   *pbState = VPC3_GET_NEXT_DXB_OUT_BUFFER_CMD();

   switch( VPC3_GET_DXB_OUT_BUFFER_SM() )
   {
      case 0x10:
      {
         pToDxbBuffer = pDpSystem->pDxbOutBuffer1;
         break;
      } /* case 0x10: */

      case 0x20:
      {
         pToDxbBuffer = pDpSystem->pDxbOutBuffer2;
         break;
      } /* case 0x20: */

      case 0x30:
      {
         pToDxbBuffer = pDpSystem->pDxbOutBuffer3;
         break;
      } /* case 0x30: */

      default:
      {
         pToDxbBuffer = VPC3_NULL_PTR;
         break;
      } /* default: */
   } /* switch( VPC3_GET_DXB_OUT_BUFFER_SM() ) */

   return pToDxbBuffer;
} /* VPC3_UNSIGNED8_PTR VPC3_GetDxbBufPtr( MEM_UNSIGNED8_PTR pbState ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_CheckDxbLinkTable                                  */
/*-------------------------------------------------------------------*/
static DP_ERROR_CODE VPC3_CheckDxbLinkTable( uint8_t bNrOfLinks )
{
DP_ERROR_CODE  bRetValue;
uint8_t          i;

   bRetValue = DP_OK;

   for( i = 0; i < bNrOfLinks; i++ )
   {
      if(     ( pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherAddress > 125 )
          ||  ( pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherLength  < 1   )
          ||  ( pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherLength  > 244 )
          ||  ( pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleOffset     > ( pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherLength - 1 ) )
          || (( pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleOffset + pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleLength) >
                                                                              (pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherLength))
          ||  ( pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleLength > MAX_DATA_PER_LINK-2 )
        )
      {
         bRetValue = DP_PRM_DXB_ERROR;
         break;
      } /* if(     ( pDpSystem->sLinkTable.sLinkTableEntry[i].bPublisherAddress > 125 ) ... */
   } /* for( i = 0; i < bNrOfLinks; i++ ) */

   return bRetValue;
} /* static DP_ERROR_CODE VPC3_CheckDxbLinkTable( uint8_t bNrOfLinks ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_BuildDxbLinkStatus                                 */
/*-------------------------------------------------------------------*/
static void VPC3_BuildDxbLinkStatus( uint8_t bNrOfLinks )
{
DXB_LINK_STATUS   sLinkStatus;
uint8_t           i;

   memset( &sLinkStatus, 0, sizeof( DXB_LINK_STATUS ) );

   sLinkStatus.bHeader     = 4 + ( bNrOfLinks * 2 );
   sLinkStatus.bStatusType = 0x83;
   sLinkStatus.bSlotNr     = 0x00;
   sLinkStatus.bSpecifier  = 0x00;

   for( i = 0; i < bNrOfLinks; i++ )
   {
      sLinkStatus.asLinkStatus[i].bPublisherAddress = pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherAddress;
      sLinkStatus.asLinkStatus[i].bLinkStatus       = 0x00;
   } /* for( i = 0; i < bNrOfLinks; i++ ) */

   VPC3_SET_DXB_LINK_STATUS_LEN( sLinkStatus.bHeader );
   CopyToVpc3_( VPC3_GET_DXB_LINK_STATUS_BUF_PTR(), (MEM_UNSIGNED8_PTR)&sLinkStatus, sLinkStatus.bHeader );
} /* static void VPC3_BuildDxbLinkStatus( uint8_t bNrOfLinks ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_SubscriberToLinkTable                              */
/*-------------------------------------------------------------------*/
DP_ERROR_CODE VPC3_SubscriberToLinkTable( MEM_PRM_SUBSCRIBER_TABLE_PTR psSubsTable, uint8_t bNrOfLinks )
{
DP_ERROR_CODE  bRetValue;
uint8_t        i;

   memset( &pDpSystem->sLinkTable, 0, sizeof( PRM_DXB_LINK_TABLE ) );

   pDpSystem->sLinkTable.bVersion = psSubsTable->bVersion;

   for( i = 0; i < bNrOfLinks; i++ )
   {
      pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherAddress  = psSubsTable->asSubscriberTableEntry[i].bPublisherAddress;
      pDpSystem->sLinkTable.asLinkTableEntry[i].bPublisherLength   = psSubsTable->asSubscriberTableEntry[i].bPublisherLength;
      pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleOffset      = psSubsTable->asSubscriberTableEntry[i].bSampleOffset;
      pDpSystem->sLinkTable.asLinkTableEntry[i].bSampleLength      = psSubsTable->asSubscriberTableEntry[i].bSampleLength;
   } /* for( i = 0; i < bNrOfLinks; i++ ) */

   bRetValue = VPC3_CheckDxbLinkTable( bNrOfLinks );

   if( bRetValue == DP_OK )
   {
      VPC3_SET_DXB_LINK_TABLE_LEN( bNrOfLinks << 2 );
      CopyToVpc3_( VPC3_GET_DXB_LINK_TABLE_BUF_PTR(), &pDpSystem->sLinkTable.asLinkTableEntry[0].bPublisherAddress, ( bNrOfLinks << 2 ) );
      VPC3_BuildDxbLinkStatus( bNrOfLinks );
   } /* if( bRetValue == DP_OK ) */

   return bRetValue;
} /* DP_ERROR_CODE VPC3_SubscriberToLinkTable( MEM_PRM_SUBSCRIBER_TABLE_PTR psSubsTable, uint8_t bNrOfLinks ) */
#endif /* #if DP_SUBSCRIBER */

/*---------------------------------------------------------------------------*/
/* function: VPC3_GetMasterAddress                                           */
/*---------------------------------------------------------------------------*/
uint8_t VPC3_GetMasterAddress( void )
{
uint8_t bMasterAddress;

   #if VPC3_SERIAL_MODE

      bMasterAddress = Vpc3Read( (VPC3_ADR)pDpSystem->pDiagBuffer1 + 3 );

      if( bMasterAddress == 0xFF )
      {
         bMasterAddress = Vpc3Read( (VPC3_ADR)pDpSystem->pDiagBuffer2 + 3 );
      } /* if( bMasterAddress == 0xFF ) */

   #else

      bMasterAddress = *( pDpSystem->pDiagBuffer1 + 3 );

      if( bMasterAddress == 0xFF )
      {
         bMasterAddress = *( pDpSystem->pDiagBuffer2 + 3 );
      } /* if( bMasterAddress == 0xFF ) */

   #endif /* #if VPC3_SERIAL_MODE */

   return( bMasterAddress );
} /* uint8_t VPC3_GetMasterAddress( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_SetIoDataLength                                            */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set length of VPC3+ input buffer and output buffer.

*/
void VPC3_SetIoDataLength( void )
{
   #if VPC3_SERIAL_MODE

      /* length of buffers OK, set real buffers */
      Vpc3Write( bVpc3RwLenDoutBuf, pDpSystem->bOutputDataLength );
      Vpc3Write( bVpc3RwLenDinBuf, pDpSystem->bInputDataLength );

   #else

      /* length of buffers OK, set real buffers */
      pVpc3->bLenDoutBuf VPC3_EXTENSION = pDpSystem->bOutputDataLength;
      pVpc3->bLenDinBuf  VPC3_EXTENSION = pDpSystem->bInputDataLength;

   #endif /* #if VPC3_SERIAL_MODE */
} /* void VPC3_SetIoDataLength( void ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_GetErrorCounter                                    */
/*-------------------------------------------------------------------*/
/*!
  \brief Get error counter from VPC3+.

  \param[in] wFalse buffer for VPC3+ error counter
  \param[in] wValid buffer for VPC3+ error counter
*/
void VPC3_GetErrorCounter( uint16_t *wFalse, uint16_t *wValid )
{
uint8_t abTemp[4];

   CopyFromVpc3_( &abTemp[0], VPC3_GET_ERROR_COUNTER_PTR(), 4 );
   *wFalse = abTemp[0] + (((uint16_t)abTemp[1]) << 8);
   *wValid = abTemp[2] + (((uint16_t)abTemp[3]) << 8);
} /* void VPC3_GetErrorCounter( uint16_t *wFalse, uint16_t *wValid ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_CopyErrorCounter                                   */
/*-------------------------------------------------------------------*/
/*!
  \brief Copy error counter from VPC3+.

  \param[in] pbData buffer for VPC3+ error counter
*/
void VPC3_CopyErrorCounter( MEM_UNSIGNED8_PTR pbData )
{
uint8_t abTemp[4];

   CopyFromVpc3_( &abTemp[0], VPC3_GET_ERROR_COUNTER_PTR(), 4 );
   pbData[0] = abTemp[1];
   pbData[1] = abTemp[0];
   pbData[2] = abTemp[3];
   pbData[3] = abTemp[2];
} /* void VPC3_CopyErrorCounter( MEM_UNSIGNED8_PTR pbData ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_WaitForGoOffline                                           */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set GoOffline and wait until VPC3+ is in offline.

  \retval DP_OK - VPC3+ is in OFFLINE state
  \retval DP_NOT_OFFLINE_ERROR - Error VPC3 is not in OFFLINE state
*/
DP_ERROR_CODE VPC3_WaitForGoOffline( void )
{
DP_ERROR_CODE bError;
uint8_t bLoop;

   VPC3_GoOffline();
   bError = DP_NOT_OFFLINE_ERROR;
   bLoop = 0;
   while( bLoop++ < 10 )
   {
      if( !VPC3_GET_OFF_PASS() )
      {
         bError = DP_OK;
         break;
      }
   } /* while( bLoop++ < 10 ) */

   return bError;
} /* DP_ERROR_CODE VPC3_WaitForGoOffline( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_GetDiagBufPtr                                              */
/*---------------------------------------------------------------------------*/
/*!
  \brief Fetch buffer pointer of the next diagnostic buffer.

  \param

  \retval !0 - pointer to current diagnostic buffer.
  \retval 0 - no buffer available
*/
VPC3_UNSIGNED8_PTR VPC3_GetDiagBufPtr( void )
{
VPC3_UNSIGNED8_PTR pToDiagBuffer;               /* pointer to diagnosis buffer */
uint8_t            bDiagBufferSm;

   bDiagBufferSm = VPC3_GET_DIAG_BUFFER_SM();

   if( ( bDiagBufferSm & 0x03 ) == 0x01 )       /* locate Diag Buffer */
   {
      pToDiagBuffer = pDpSystem->pDiagBuffer1;
   } /* if( ( bDiagBufferSm & 0x03 ) == 0x01 ) */
   else
   {
      if( ( bDiagBufferSm & 0x0C ) == 0x04 )
      {
         pToDiagBuffer = pDpSystem->pDiagBuffer2;
      } /* if( ( bDiagBufferSm & 0x0C ) == 0x04 ) */
      else
      {
         pToDiagBuffer = VPC3_NULL_PTR;
      } /* else of if( ( bDiagBufferSm & 0x0C ) == 0x04 ) */
   } /* else of if( ( bDiagBufferSm & 0x03 ) == 0x01 ) */

   return pToDiagBuffer;
} /* VPC3_UNSIGNED8_PTR VPC3_GetDiagBufPtr( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_UpdateDiagnosis                                            */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set diagnostic data to VPC3+.
  By calling this function, the user transfers the new, external diagnostics to VPC3+.
  As a return value, the user receives a pointer to the new diagnostics data buffer.
  The user has to make sure that the buffer size does not exceed the size of the
  diagnostic buffer that was set when the slave’s memory resources were set up:

  Diagnostic Buffer Length >= bUserDiagLen + DIAG_NORM_DIAG_SIZE

  \param[in] bDiagControl - add diagnostic bits
  \param[in] pbToUserDiagData - pointer to structure with alarm/diagnostic data
  \param[in] bUserDiagLength - length of the current user diagnostic

  Values for bUserDiagLength:
  - 0 - A previously set user diagnostic is deleted from the slave’s diagnostic buffer. Only 6 bytes
        standard diagnostic are sent in the diagnostic telegram. In this case, the user does not have to transfer a
        pointer to a diagnostic buffer.
  - 1..DIAG_BUFSIZE-6 - Length of the new user diagnostic data.

  Values for bDiagControl:
  - DIAG_RESET - Reset bit 'extended diagnostic, static diagnostic, diagnostic overflow'
  - EXT_DIAG_SET - Set bit 'extended diagnostic'.
  - STAT_DIAG_SET - Set bit 'static diagnostic'.

  \retval !0 - pointer to current diagnostic buffer.
  \retval 0 - no buffer available
*/
static VPC3_UNSIGNED8_PTR VPC3_UpdateDiagnosis( uint8_t bDiagControl, MEM_UNSIGNED8_PTR pbToUserDiagData, uint8_t bUserDiagLen )
{
VPC3_UNSIGNED8_PTR pDiagBuffer; /* pointer to diagnosis buffer */
uint8_t            bNewDiagBufferCmd;
uint8_t            bDiagBufferSm;

   bDiagBufferSm = VPC3_GET_DIAG_BUFFER_SM();

   if( ( bDiagBufferSm & 0x03 ) == 0x01 )                      /* locate Diag Buffer */
   {
      /* copy to diagnosis buffer */
      if( bUserDiagLen > 0 )
      {
         CopyToVpc3_( pDpSystem->pDiagBuffer1+DIAG_NORM_DIAG_SIZE, pbToUserDiagData, bUserDiagLen );
      } /* if( bUserDiagLen > 0 ) */

      #if VPC3_SERIAL_MODE
         Vpc3Write( bVpc3RwLenDiagBuf1, bUserDiagLen+DIAG_NORM_DIAG_SIZE );
         Vpc3Write( (VPC3_ADR)pDpSystem->pDiagBuffer1, bDiagControl );
      #else
         pVpc3->abLenDiagBuf[0] VPC3_EXTENSION = bUserDiagLen+DIAG_NORM_DIAG_SIZE;     /* length of Diag Buffer 1 */
         *(pDpSystem->pDiagBuffer1) = bDiagControl;
      #endif /* #if VPC3_SERIAL_MODE */
   } /* if( ( bDiagBufferSm & 0x03 ) == 0x01 ) */
   else
   {
      if( ( bDiagBufferSm & 0x0C ) == 0x04 )
      {
         /* copy to diagnosis buffer */
         if( bUserDiagLen > 0 )
         {
            CopyToVpc3_( pDpSystem->pDiagBuffer2+DIAG_NORM_DIAG_SIZE, pbToUserDiagData, bUserDiagLen );
         } /* if( bUserDiagLen > 0 ) */

         #if VPC3_SERIAL_MODE
            Vpc3Write( bVpc3RwLenDiagBuf2, bUserDiagLen+DIAG_NORM_DIAG_SIZE );
            Vpc3Write( (VPC3_ADR)pDpSystem->pDiagBuffer2, bDiagControl );
         #else
            pVpc3->abLenDiagBuf[1] VPC3_EXTENSION = bUserDiagLen+DIAG_NORM_DIAG_SIZE;  /* length of Diag Buffer 2 */
            *(pDpSystem->pDiagBuffer2) = bDiagControl;
         #endif /* #if VPC3_SERIAL_MODE */
      } /* if( ( bDiagBufferSm & 0x0C ) == 0x04 ) */
   } /* else of if( ( bDiagBufferSm & 0x03 ) == 0x01 ) */

   bNewDiagBufferCmd = VPC3_GET_NEW_DIAG_BUFFER_CMD();

   switch( bNewDiagBufferCmd & 0x03 )
   {
      case 1:   /* use buffer 1 */
      {
         pDiagBuffer = pDpSystem->pDiagBuffer1;
         break;
      } /* case 1: */

      case 2:   /* use buffer 2 */
      {
         pDiagBuffer = pDpSystem->pDiagBuffer2;
         break;
      } /* case 2: */

      default:
      {
         /* no buffer available */
         pDiagBuffer = VPC3_NULL_PTR;
         break;
      } /* default: */
   } /* switch( bNewDiagBufferCmd & 0x03 ) */

   return pDiagBuffer;
} /* static VPC3_UNSIGNED8_PTR VPC3_UpdateDiagnosis( uint8_t bDiagControl, MEM_UNSIGNED8_PTR pbToUserDiagData, uint8_t bUserDiagLen ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_CheckDiagBufPtr                                            */
/*---------------------------------------------------------------------------*/
static VPC3_UNSIGNED8_PTR VPC3_CheckDiagBufPtr( void )
{
   if( pDpSystem->pDiagBuffer == VPC3_NULL_PTR )
   {
      pDpSystem->pDiagBuffer = VPC3_GetDiagBufPtr();
   }
   return pDpSystem->pDiagBuffer;
} /* static VPC3_UNSIGNED8_PTR VPC3_CheckDiagBufPtr( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_CheckDiagBufferChanged                                     */
/*---------------------------------------------------------------------------*/
void VPC3_CheckDiagBufferChanged( void )
{
uint8_t bVpc3Event;

   if( VPC3_GetDpState( eDpStateDiagActive ) )
   {
      #if VPC3_SERIAL_MODE
         bVpc3Event = Vpc3Read( bVpc3RwIntReqReg_H );
      #else
         bVpc3Event = pVpc3->bIntReqReg_H VPC3_EXTENSION;
      #endif /* #if VPC3_SERIAL_MODE */
      if( bVpc3Event & VPC3_INT_DIAG_BUFFER_CHANGED )
      {
         DpDiag_IsrDiagBufferChanged();
         #if VPC3_SERIAL_MODE
            Vpc3Write( bVpc3WoIntAck_H, VPC3_INT_DIAG_BUFFER_CHANGED );
         #else
            pVpc3->sReg.sWrite.bIntAck_H VPC3_EXTENSION = VPC3_INT_DIAG_BUFFER_CHANGED;
         #endif /* #if VPC3_SERIAL_MODE */
      }
   } /* if( VPC3_GetDpState( eDpStateDiagActive ) ) */
} /* void VPC3_CheckDiagBufferChanged( void ) */

/*---------------------------------------------------------------------------*/
/* function: VPC3_SetDiagnosis                                               */
/*---------------------------------------------------------------------------*/
/*!
  \brief Set diagnostic data to VPC3+.
  By calling this function, the user provides diagnostic data to the slave. The diagnostic
  data is sent at the next possible time.
  The user has to make sure that the buffer size does not exceed the size of the
  diagnostic buffer that was set when the slave’s memory resources were set up:

  Diagnostic Buffer Length >= LengthDiag_User + LengthAlarm_User

  \attention This function is not suitable for setting alarms.

  The user can set up DP diagnostics; the following applies:
   - The 6 byte standard diagnostic (refer to EN 50170) is not part of the user diagnostic.
   - In DP standard operation, one ID-related, several channel-related, and one
   - device-related diagnostics may be utilized.
   - In DPV1 operation, one revision, one ID-related, several channel-related, and
     one alarm.
   - The user is responsible for the content of the diagnostic data.

  \param[in] pbToUserDiagData - pointer to structure with alarm/diagnostic data
  \param[in] bUserDiagLength - length of the current user diagnostic
  \param[in] bDiagControl - add diagnostic bits
  \param[in] bCheckDiagFlag - check VPC3+ diagnostic flag

  Values for bUserDiagLength:
  - 0 - A previously set user diagnostic is deleted from the slave’s diagnostic buffer. Only 6 bytes
        standard diagnostic are sent in the diagnostic telegram. In this case, the user does not have to transfer a
        pointer to a diagnostic buffer.
  - 1..DIAG_BUFSIZE-6 - Length of the new user diagnostic data.

  Values for bDiagControl:
  - DIAG_RESET - Reset bit 'extended diagnostic, static diagnostic, diagnostic overflow'
  - EXT_DIAG_SET - Set bit 'extended diagnostic'.
  - STAT_DIAG_SET - Set bit 'static diagnostic'.

  \retval DP_OK - Execution OK, diagnostic message is copied into VPC3+
  \retval DP_DIAG_OLD_DIAG_NOT_SEND_ERROR - Error, wait because last diagnostic message isn't send
  \retval DP_DIAG_BUFFER_LENGTH_ERROR - Error, diagnostic message is too long
  \retval DP_DIAG_NO_BUFFER_ERROR - Error, no diagnostic buffer available
  \retval DP_DIAG_CONTROL_BYTE_ERROR - Error of bDiagControl
  \retval DP_DIAG_BUFFER_ERROR - Error, diagnostic header is wrong
  \retval DP_DIAG_SEQUENCE_ERROR - Error, revision will be send in wrong state
  \retval DP_DIAG_NOT_POSSIBLE_ERROR - Error, unknown diagnostic header
*/
DP_ERROR_CODE VPC3_SetDiagnosis( MEM_UNSIGNED8_PTR pbToUserDiagData, uint8_t bUserDiagLength, uint8_t bDiagControl, uint8_t bCheckDiagFlag )
{
MEM_UNSIGNED8_PTR pbToDiagArray;
DP_ERROR_CODE     bRetValue;
uint8_t           bTmpUserDiagnosisLength;
uint8_t           bTmpLength;
uint8_t           bHeader;
uint8_t           bDpState;

   bRetValue = DP_OK;

   /* check available diag buffer */
   if( VPC3_CheckDiagBufPtr() != VPC3_NULL_PTR )
   {
      bTmpUserDiagnosisLength = bUserDiagLength;
      pbToDiagArray = pbToUserDiagData;

      bDpState = VPC3_GET_DP_STATE();
      if( ( bDpState == DATA_EX ) && ( bCheckDiagFlag == VPC3_TRUE ) )
      {
         if( VPC3_GET_DIAG_FLAG() )
         {
            /* old diagnosis not send */
            bRetValue = DP_DIAG_OLD_DIAG_NOT_SEND_ERROR;
         } /* if( VPC3_GET_DIAG_FLAG() ) */
      } /* if( ( bDpState == DATA_EX ) && ( bCheckDiagFlag == VPC3_TRUE ) ) */

      /* check bUserDiagLength */
      if( bUserDiagLength > ( DIAG_BUFSIZE - 6 ) )
      {
         bRetValue = DP_DIAG_BUFFER_LENGTH_ERROR;
      } /* if( bUserDiagLength > ( DIAG_BUFSIZE - 6 ) ) */

      if( bRetValue == DP_OK )
      {
         /* check control byte */
         switch( bDiagControl )
         {
            case EXT_DIAG_SET:
            {
               if( bUserDiagLength == 0 )
               {
                   bRetValue = DP_DIAG_CONTROL_BYTE_ERROR;
               } /* if( bUserDiagLength == 0 ) */
               break;
            } /* case EXT_DIAG_SET: */

            default:
            {
               /* do nothing */
               break;
            } /* default: */
         } /* switch( bDiagControl ) */

         /* check user diag buffer contents */
         while( ( 0 < bTmpUserDiagnosisLength ) && ( DP_OK == bRetValue ) )
         {
            bHeader = pbToDiagArray[0];
            switch( DIAG_TYPE_MASK & bHeader )
            {
               case DIAG_TYPE_DEV:
               {
                  bTmpLength = (( ~DIAG_TYPE_MASK ) & bHeader );
                  if( STATUS_DIAG_HEAD_SIZE > bTmpLength )
                  {
                     bRetValue = DP_DIAG_BUFFER_ERROR;
                  } /* if( STATUS_DIAG_HEAD_SIZE > bTmpLength ) */
                  break;
               } /* case DIAG_TYPE_DEV: */

               case DIAG_TYPE_KEN:
               {
                  bTmpLength = (( ~DIAG_TYPE_MASK ) & bHeader );
                  if ( bTmpLength == 0 )
                  {
                     bRetValue = DP_DIAG_BUFFER_ERROR;
                  } /* if ( bTmpLength == 0 ) */
                  break;
               } /* case DIAG_TYPE_KEN: */

               case DIAG_TYPE_CHN:
               {
                  bTmpLength = DIAG_TYPE_CHN_SIZE;
                  break;
               } /* case DIAG_TYPE_CHN: */

               case DIAG_TYPE_REV:
               {
                  bTmpLength = DIAG_TYPE_REV_SIZE;
                  if( bDpState != DATA_EX )
                  {
                     /* only allowed in state DATA_EX */
                     bRetValue = DP_DIAG_SEQUENCE_ERROR;
                  } /* if( bDpState != DATA_EX ) */
                  break;
               } /* case DIAG_TYPE_REV: */

               default:
               {
                  /* not possible! */
                  bTmpLength = 0;
                  bRetValue = DP_DIAG_NOT_POSSIBLE_ERROR;
                  break;
               } /* default: */
            } /* switch( DIAG_TYPE_MASK & bHeader ) */

            bTmpUserDiagnosisLength -= bTmpLength;
            pbToDiagArray += bTmpLength;
         } /* while( ( 0 < bTmpUserDiagnosisLength ) && ( DP_OK == bRetValue ) ) */

         if( bRetValue == DP_OK )
         {
            pDpSystem->pDiagBuffer = VPC3_UpdateDiagnosis( bDiagControl, pbToUserDiagData, bUserDiagLength );
         } /* if( bRetValue == DP_OK ) */
      } /* if( bRetValue == DP_OK ) */
   } /* if( VPC3_CheckDiagBufPtr() != VPC3_NULL_PTR ) */
   else
   {
      /* Fetch new diagnosis buffer */
      pDpSystem->pDiagBuffer = VPC3_GetDiagBufPtr();
      /* wait for next free diag_buffer */
      bRetValue = DP_DIAG_NO_BUFFER_ERROR;
   } /* else of if( VPC3_CheckDiagBufPtr() != VPC3_NULL_PTR ) */

   return bRetValue;
} /* DP_ERROR_CODE VPC3_SetDiagnosis( MEM_UNSIGNED8_PTR pbToUserDiagData, uint8_t bUserDiagLength, uint8_t bDiagControl, uint8_t bCheckDiagFlag ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_SetAlarm                                           */
/*-------------------------------------------------------------------*/
/*!
  \brief Set alarm to VPC3+.
  By calling this function, the stack accepts the transferred alarm data. In addition to the net
  data, the alarm data also includes control information according to the DPV1
  specification . The data is transmitted at the next possible time. The user has to make sure that
  the buffer size does not exceed the size of the diagnostic buffer that was set when the
  slave’s memory resources were defined.

  Diagnostic Buffer Length >= LengthDiag_User + LengthAlarm_User

  Specifications:
   - When setting alarms, the user has to adhere to the requirements regarding
     permissible alarm types that he was informed of when the alarm state machine was
     started.
   - The number of alarms that are permitted to be processed simultaneously during
     communication between parameterization master and slave is specified by the
     type- or sequence mode. It is entirely handled automatically by the stack; the user has no
     influence on it, and can thus set any number of alarms of all permitted types.
   - The user is responsible for the content of the alarm data.
   - The alarm buffer is to contain only one alarm.
   - The user has possibility to add DPV0 diagnostics like identifier related, modulestatus, etc. via the
     function DpDiag_Alarm.

  \param[in] psAlarm - pointer to structure with alarm data
  \param[in] bCallback - VPC3_TRUE - the stack calls the function UserAlarm to add diagnostics like channel related / identifier related diagnostics.
  \param[in] bCallback - VPC3_FALSE - the stack sends directly alarm data

  \retval SET_ALARM_OK - Execution OK, alarm add to alarm state machine
  \retval SET_ALARM_AL_STATE_CLOSED - Error, alarm state machine is closed
  \retval SET_ALARM_ALARMTYPE_NOTSUPP - Error, alarm type is not supported
  \retval SET_ALARM_SEQ_NR_ERROR - Error, sequence number is wrong
  \retval SET_ALARM_SPECIFIER_ERROR - Error, alarm specifier is wrong
*/
uint8_t VPC3_SetAlarm( ALARM_STATUS_PDU_PTR psAlarm, uint8_t bCallback )
{
   #if DP_ALARM

      return AL_SetAlarm( psAlarm, bCallback );

   #else

      psAlarm   = psAlarm;    /* avoid warning */
      bCallback = bCallback;  /* avoid warning */

      return SET_ALARM_AL_STATE_CLOSED;

   #endif /* #if DP_ALARM */
} /* uint8_t VPC3_SetAlarm( ALARM_STATUS_PDU_PTR psAlarm, uint8_t bCallback ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_GetFreeMemory                                      */
/*-------------------------------------------------------------------*/
/*!
   \brief Determine free memory space.

   \retval free memory size.
*/
uint16_t VPC3_GetFreeMemory( void )
{
uint16_t wFreeMemorySize;

   /* return number of bytes of unused VPC3-ram */
   wFreeMemorySize = ( ASIC_USER_RAM - pDpSystem->wVpc3UsedDPV0BufferMemory - pDpSystem->wVpc3UsedDPV1BufferMemory );

   return wFreeMemorySize;
} /* uint16_t VPC3_GetFreeMemory( void ) */

/*-------------------------------------------------------------------*/
/* function: VPC3_ProcessDpv1StateMachine                            */
/*-------------------------------------------------------------------*/
/*!
  \brief The application program has to call this function cyclically so that the DPV1 services can be processed.
*/
void VPC3_ProcessDpv1StateMachine( void )
{
   #if DP_MSAC_C1
      MSAC_C1_Process();   /* state machine MSAC_C1 */
   #endif /* #if DP_MSAC_C1 */

   #if DP_ALARM
      AL_AlarmProcess();   /* state machine ALARM */
   #endif /* #if DP_ALARM */

   #if DP_MSAC_C2
      MSAC_C2_Process();   /* state machine MSAC_C2 */
   #endif /* #if DP_MSAC_C2 */
} /* void VPC3_ProcessDpv1StateMachine( void ) */

/*****************************************************************************/
/*  Copyright (C) profichip GmbH 2009. Confidential.                         */
/*****************************************************************************/

