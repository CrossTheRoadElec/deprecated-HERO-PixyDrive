/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) binary firmware files (*.crf) 
 * and software example source ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System.Collections;

namespace CTRE
{
    namespace HERO
    {
        namespace Module
        {
            public class PixyCamera : ModuleBase
            {
                /** States for Pixy processing */
                private enum States
                {
                    FirstWord,
                    SecondWord,
                    Block,
                    FirstWordContinous,
                }
                private States PixyState;

                /** Sync Frame */
                private const UInt16 Pixy_FRAME_START_WORD = 0xAA55;
                /** Out of order Frame */
                private const UInt16 Pixy_FRAME_WRONG_ORDER = 0x55AA;
                /** ColorCode Block Sync Frame */
                private const UInt16 Pixy_FRAME_START_WORD_CC = 0xAA56;

                /** Data Sync Byte */
                private const byte Pixy_SYNC_BYTE = 0x5A;       /* 01011010 */
                /** Data Send/Sync Byte */
                private const byte Pixy_SYNC_BYTE_DATA = 0x5B;  /* 01011011 */

                /** Circular Queue for Collecting Data */
                private byte[] OutBuffer;
                private int OutLength = 0;
                private int OutReadIndex = 0;
                private int OutWriteIndex = 0;
                private const int Pixy_OUTBUFFER_SIZE = 64;

                private SPI PixyCam_SPI;
                /** Tracks if we have gotten a color coded block */
                private bool IsColorCoded = false;
                /** String for Pixy */
                public System.Text.StringBuilder Bytes = new System.Text.StringBuilder();
                /** Holds the collected blocks, each element is a block */
                private ArrayList _Blocks = new ArrayList();

                /** Variable for holding words from MISO */
                private UInt16 Word = 0;
                /** Used to compare to checksum for data validity */
                private UInt16 Sum = 0;
                /** Checksum grabbed form second word of block */
                private UInt16 Checksum = 0; 
                /** How many blocks are avaiable for grab */
                private UInt16 _BlockCount = 0;
                /** Block that we store all our BlockData */
                private PixyBlock Block;
                /** Temporary Chip Select for debugging */
                OutputPort TempCS = new OutputPort(CTRE.HERO.IO.Port8.Pin5, true);
                /** Intiate value for tracking time */
                DateTime Start;

                /** 
                 * Initate Status values and creates a string to print upon request
                 */
                public class PixyStatus
                {
                    public bool Synced = false;
                    public UInt16 SyncErrorCount = 0;
                    public UInt16 ChecksumErrorCount = 0;
                    public long TimeSinceLastBlock = 0;

                    public override string ToString()
                    {
                        return "Sync: " + Synced + " Unsync Count: " + SyncErrorCount + " ChecksumErrorCount: " + ChecksumErrorCount + " TimeSinceLastBlock: " + TimeSinceLastBlock;
                    }
                }
                PixyStatus _pixyStatus = new PixyStatus();

                /** 
                 * Getter that returns status values as well as tells us time since last block and send CAN Frames for debugging
                 */
                public PixyStatus Status
                {
                    get
                    {
                        /* Grab ticks */
                        _pixyStatus.TimeSinceLastBlock = DateTime.Now.Subtract(Start).Ticks;
                        /* Turn ticks into milliseconds */
                        _pixyStatus.TimeSinceLastBlock /= TimeSpan.TicksPerMillisecond;

                        /* Send frames of Pixy Status */
                        byte[] Frame = new byte[8];
                        Frame[0] = (byte)(_pixyStatus.SyncErrorCount >> 8);
                        Frame[1] = (byte)(_pixyStatus.SyncErrorCount & 0xFF);
                        Frame[2] = (byte)(_pixyStatus.ChecksumErrorCount >> 8);
                        Frame[3] = (byte)(_pixyStatus.ChecksumErrorCount & 0xFF);
                        Frame[4] = (byte)(_pixyStatus.TimeSinceLastBlock >> 8);
                        Frame[5] = (byte)(_pixyStatus.TimeSinceLastBlock & 0xFF);
                        Frame[6] = (_pixyStatus.Synced ? (byte)1 : (byte)0);
                        ulong data = (ulong)BitConverter.ToInt64(Frame, 0);
                        CTRE.Native.CAN.Send(41, data, 8, 0);       /* Address is 29 in vehicle spi */

                        return _pixyStatus;
                    }
                }

                /**
                 * PixyCam Constructor
                 * 
                 * Also Initializes OutputBuffer and Sets PixyState to FirstWord
                 * 
                 * @param   PortDef     Port PixyCam is plugged into
                 * @param   ClockRate   Clockrate of SPI
                 */
                public PixyCamera(Port8Definition PortDef, UInt16 ClockRate)
                {
                    /* Constructs PixyCam */
                    SPI.Configuration PixyCam_SPIConfig = new SPI.Configuration(HERO.IO.Port8.Pin6, false, 0, 100, false, true, ClockRate, SPI.SPI_module.SPI4);
                    PixyCam_SPI = new SPI(PixyCam_SPIConfig);

                    /* Initialize Output Buffer */
                    OutBuffer = new byte[Pixy_OUTBUFFER_SIZE];
                    /* Initialze Pixy Processing State */
                    PixyState = States.FirstWord;
                }

                /**
                 * Single byte read and Write
                 * 
                 * @param   DataInTheOutQueue   Data to be sent when Reading/Writing
                 * @return  Buffer[0]           Returns a single byte of data (part of word)
                 */
                private byte GetByte(byte DataInTheOutQueue)
                {
                    TempCS.Write(false);
                    byte[] buffer = new byte[1];
                    PixyCam_SPI.WriteRead(new byte[] { DataInTheOutQueue }, buffer);
                    TempCS.Write(true);
                    return buffer[0];
                }

                /**
                 * Single Word Read
                 * 
                 * @return Word  2 Bytes of data combined for a single word
                 */
                private UInt16 GetWord()
                {
                    /* Word for return */
                    UInt16 Word = 0;
                    /* Holds part of the word */
                    byte Temp;
                    /* No data to send or just 0's */
                    byte Filler = 0;

                    if (OutLength != 0)
                    {
                        /* There is data in the output buffer */
                        Word = GetByte(Pixy_SYNC_BYTE_DATA);        /* Pull the data */
                        Filler = OutBuffer[OutReadIndex++];         /* Store Output data into Filler */
                        OutLength--;                                /* Output data decreased by 1 */
                        if (OutReadIndex == Pixy_OUTBUFFER_SIZE)    /* Checks to see if we are full */
                            OutReadIndex = 0;                       /* Full so put at the beginning */
                    }
                    else
                    {
                        Word = GetByte(Pixy_SYNC_BYTE);     /* Grab the first byte of the word */
                    }

                    Word = (UInt16)(((UInt16)Word) << 8);   /* Shift 2nd byte to 1st byte */
                    Temp = GetByte(Filler);                 /* Send data out and pull 2nd byte */
                    Word |= Temp;                           /* Inlude the 2nd byte  in the word */

                    return Word;
                }

                /** 
                 * Queues data to be sent when GetWord() is called
                 * 
                 * @param   data    Byte of data to send on MOSI
                 * @return  Length  if there is anything send or if there is any data available to send
                 */
                private bool Send(byte[] data)
                {
                    int Length = data.Length;

                    if (OutLength + Length > Pixy_OUTBUFFER_SIZE || OutLength != 0)
                        return false;

                    OutLength += Length;
                    for (int i = 0; i < Length; i++)
                    {
                        OutBuffer[OutWriteIndex++] = data[i];
                        if (OutWriteIndex == Pixy_OUTBUFFER_SIZE)
                            OutWriteIndex = 0;
                    }
                    return (Length > 0);
                }

                /**
                 * Grabs the number of blocks
                 * 
                 * @return  _Blocks.Count   Number of blocks available 
                 */
                public uint BlockCount
                {
                    get
                    {
                        return (uint)_Blocks.Count;
                    }
                }
                
                /**
                 * Grabs a PixyBlock
                 * 
                 * @param   blockToFill     Block to fill good with
                 * @return  bool            If Block data is available or not
                 */
                public bool GetBlock(PixyBlock blockToFill)
                {
                    if (_Blocks.Count > 0)
                    {
                        PixyBlock pb = (PixyBlock)_Blocks[0]; /* TODO:replace circulator buffer */
                        pb.CopyTo(blockToFill);
                        _Blocks.RemoveAt(0); /* TODO: replace circulator buffer */
                        return true;
                    }
                    return false;
                }

                /** 
                 * Processing that searches for the beginning of the frame and syncs up to grab data
                 */
                public void Process()
                {
                    switch (PixyState)
                    {
                        case States.FirstWord:
                            /* Find the start of the frame */
                            Word = GetWord();                   /* Grab Word */
                            if (Word == Pixy_FRAME_START_WORD)
                            { 
                                PixyState = States.SecondWord;  /* First word good, look for second word */
                            }
                            else if (Word == Pixy_FRAME_WRONG_ORDER)
                            {
                                GetByte(0);                     /* Immediately clock 8 bits to resync */
                                ++_pixyStatus.SyncErrorCount;   /* Track error */
                                _pixyStatus.Synced = false;     /* Lost sync */
                            }
                            else if (Word == 0)
                            {
                                /* Do nothing because Pixy is synced */
                            }
                            else
                            {
                                ++_pixyStatus.SyncErrorCount;   /* Track error */
                                _pixyStatus.Synced = false;     /* Lost sync */
                            }
                            break;

                        case States.SecondWord:
                            //Find the second start frame and determie if Single color or Color coded
                            Word = GetWord();                   /* Grab Word */
                            if (Word == Pixy_FRAME_START_WORD)
                            {
                                /* Frame Start found and single color block */
                                IsColorCoded = false;
                                PixyState = States.Block;
                            }
                            else if (Word == Pixy_FRAME_START_WORD_CC)
                            {
                                /* Frame Start found and Colorcoded block */
                                IsColorCoded = true;
                                PixyState = States.Block;
                            }
                            else
                            {
                                /* Frame Start not found so restart entire process, not synced */
                                PixyState = States.FirstWord;   /* Restart the StartFrame Search */
                                ++_pixyStatus.SyncErrorCount;   /* Track error */
                                _pixyStatus.Synced = false;     /* Lost sync */
                            }
                            break;

                        case States.Block:
                            /* Synced, now grab Block from Pixy */
                            _pixyStatus.Synced = true;          /* Synced */
                            Start = DateTime.Now;               /* Initiate TimeSinceLastBlock */
                            Block = new PixyBlock();            /* Block for holding data */

                            Sum = 0;                            /* Reset sum */
                            Checksum = GetWord();               /* Grab checksum */
                            /* Grab Signature */
                            Block.Signature = GetWord();
                            Sum += Block.Signature;
                            /* Grab X */
                            Block.X = GetWord();
                            Sum += Block.X;
                            /* Grab Y */
                            Block.Y = GetWord();
                            Sum += Block.Y;
                            /* Grab Width */
                            Block.Width = GetWord();
                            Sum += Block.Width;
                            /* Grab Height */
                            Block.Height = GetWord();
                            Sum += Block.Height;
                            /* Check to see if we can get angle */
                            if (!IsColorCoded)
                            {
                                /*Angle only availabe in CC mode*/
                                Block.Angle = 0;
                            }
                            else
                            {
                                /* Colorcoded block so get angle */
                                Block.Angle = (Int16)GetWord();
                                Sum += (UInt16)Block.Angle;
                            }
                            /* Calculate area */
                            Block.Area = (int)(Block.Height * Block.Width);
                            /* ColorCode store into Block */
                            Block.IsColorCoded = IsColorCoded;

                            /* Send CAN Frames of the current data */
                            byte[] Frame1 = new byte[8];
                            Frame1[0] = (byte)(Block.X >> 8);
                            Frame1[1] = (byte)(Block.X & 0xFF);
                            Frame1[2] = (byte)(Block.Y >> 8);
                            Frame1[3] = (byte)(Block.Y & 0xFF);
                            Frame1[4] = (byte)(Block.Width >> 8);
                            Frame1[5] = (byte)(Block.Width & 0xFF);
                            Frame1[6] = (byte)(Block.Height >> 8);
                            Frame1[7] = (byte)(Block.Height & 0xFF);
                            ulong data1 = (ulong)BitConverter.ToInt64(Frame1, 0);
                            CTRE.Native.CAN.Send(9, data1, 8, 0);
                            /* CAN Frames part 2 */
                            byte[] Frame2 = new byte[8];
                            Frame2[0] = (byte)(Block.Signature >> 8);
                            Frame2[1] = (byte)(Block.Signature & 0xFF);
                            Frame2[2] = (byte)(Block.Angle >> 8);
                            Frame2[3] = (byte)(Block.Angle & 0xFF);
                            Frame2[4] = (byte)(Block.Area >> 8);
                            Frame2[5] = (byte)(Block.Area & 0xFF);
                            ulong data2 = (ulong)BitConverter.ToInt64(Frame2, 0);
                            CTRE.Native.CAN.Send(25, data2, 8, 0);

                            if (Checksum == Sum)
                            {
                                /* Checksum is valid so store the block */
                                _BlockCount++;
                                _Blocks.Add(Block);
                            }
                            else
                            {
                                /* Checksum is invalid, throwaway block and keep track */
                                ++_pixyStatus.ChecksumErrorCount;
                            }
                            /* Finished grabbing block, reset processing */
                            PixyState = States.FirstWord;
                            break;
                    }
                }

                /* Currently private as there is no was to set the AutoSettings in code */
                /* 
                 * Sets the brighness of the Pixy's Camera 
                 * 
                 * @param   Brightness   Value from 0 - 255
                 * @return  Data         Sends the data to be queued up
                 */
                private bool SetBrightness(byte Brightness)
                {
                    /* This function has effect only when the 'Auto Exposure Correction' option */
                    /* is selected on the camera tab of GUI's configuration window */
                    /* Brightness Sync is a word */
                    byte Pixy_CAM_BRIGHTNESS_SYNC = 0xFE;
                    byte[] Buffer = new byte[3] { 0x00, Pixy_CAM_BRIGHTNESS_SYNC, Brightness };
                    return Send(Buffer);
                }

                /*
                 * Sets the LED Color with RBG values of 0 or 255
                 * 
                 * @param   r       bool 0 or 255 of red
                 * @param   g       bool 0 or 255 of green
                 * @param   b       bool 0 or 255 of blue
                 * @return  Data    Sends the data to be queued up
                 */
                public bool SetLED(bool rLED, bool gLED, bool bLED)
                {
                    /* Stills need to be tested whether it can be values between 0 -255 or just the extremes */
                    byte r = 0;     /* red */
                    byte g = 0;     /* green */
                    byte b = 0;     /* blue */

                    if (rLED) r = 255;
                    if (gLED) g = 255;
                    if (bLED) b = 255;
                    /* LED Sync is a word */
                    byte Pixy_LED_SYNC = 0xFD;
                    byte[] Buffer = new byte[5] { 0x00, Pixy_LED_SYNC, r, g, b };
                    return Send(Buffer);
                }
            }

            /**
             * Getter for grabbing block data generated from processing and generates a string upon request
             */
            public class PixyBlock
            {
                //Bytes    16-bit words   Description
                //----------------------------------------------------------------
                //0, 1     0              sync (0xaa55)
                //2, 3     1              checksum (sum of all 16-bit words 2-6)
                //4, 5     2              signature number
                //6, 7     3              x center of object
                //8, 9     4              y center of object
                //10, 11   5              width of object
                //12, 13   6              height of object
                //14, 15   7		      angle of CC object	

                /* Straight from Pixy */
                public UInt16 Signature { get; set; }
                public UInt16 X { get; set; }  			//!< Block's top left X, 0 to 319
                public UInt16 Y { get; set; }  			//!< Block's top left Y, 0 to 199
                public UInt16 Width { get; set; }
                public UInt16 Height { get; set; }
                public Int16 Angle { get; set; }
                /* Processed data from the block */
                public int Area { get; set; }
                public bool IsColorCoded { get; set; } 

                public override string ToString()
                {
                    return "S: " + Signature + " X: " + X + " Y: " + Y + " W: " + Width + " H: " + Height + " Ang: " + Angle + " Area: " + Area + " CC: " + IsColorCoded;
                }

                /* Copies block data to another block */
                public void CopyTo(PixyBlock block)
                {
                    block.Signature = Signature;
                    block.X = X;
                    block.Y = Y;
                    block.Width = Width;
                    block.Height = Height;
                    block.Angle = Angle;
                    block.Area = Area;
                    block.IsColorCoded = IsColorCoded;
                }
            }
        }
    }
}