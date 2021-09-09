#ifndef __DCI_H
#define __DCI_H

#include <stdint.h>

///  DCI Format Type 0 (5 MHz,TDD0, 27 bits)
struct DCI0_5MHz_TDD0 {
  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
  uint32_t type:1;
  /// Hopping flag
  uint32_t hopping:1;
  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Power Control
  uint32_t TPC:2;
  /// Cyclic shift
  uint32_t cshift:3;
  /// DAI (TDD)
  uint32_t ulindex:2;
  /// CQI Request
  uint32_t cqi_req:1;
  /// Padding to get to size of DCI1A
  uint32_t padding:2;
} __attribute__ ((__packed__));

typedef struct DCI0_5MHz_TDD0 DCI0_5MHz_TDD0_t;
#define sizeof_DCI0_5MHz_TDD_0_t 27

///  DCI Format Type 0 (1.5 MHz,TDD1-6, 23 bits)
struct DCI0_1_5MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:5;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Cyclic shift
	  uint32_t cshift:3;
	  /// DAI
	  uint32_t dai:2;
	  /// CQI Request
	  uint32_t cqi_req:1;
	  /// Padding
	  uint32_t padding:11;
} __attribute__ ((__packed__));

typedef struct DCI0_1_5MHz_TDD_1_6 DCI0_1_5MHz_TDD_1_6_t;
#define sizeof_DCI0_1_5MHz_TDD_1_6_t 24

/// DCI Format Type 1A (1.5 MHz, TDD, frame 1-6, 24 bits)
struct DCI1A_1_5MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	  /// RB Assignment (ceil(log2(N_RB_DL*(N_RB_DL-1)/2)) bits)
	  uint32_t rballoc:5;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t dai:2;
	  /// SRS request bit
	  uint32_t srs_req:1;
	  /// padding
	  uint32_t padding:8;
} __attribute__ ((__packed__));

typedef struct DCI1A_1_5MHz_TDD_1_6 DCI1A_1_5MHz_TDD_1_6_t;
#define sizeof_DCI1A_1_5MHz_TDD_1_6_t 24


///  DCI Format Type 0 (5 MHz,TDD1-6, 27 bits)
struct DCI0_5MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:9;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Cyclic shift
	  uint32_t cshift:3;
	  /// DAI
	  uint32_t dai:2;
	  /// CQI Request
	  uint32_t cqi_req:1;
	  /// Padding
	  uint32_t padding:7;
} __attribute__ ((__packed__));

typedef struct DCI0_5MHz_TDD_1_6 DCI0_5MHz_TDD_1_6_t;
#define sizeof_DCI0_5MHz_TDD_1_6_t 27

/// DCI Format Type 1A (5 MHz, TDD, frame 1-6, 27 bits)
struct DCI1A_5MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL*(N_RB_DL-1)/2)) bits)
	  uint32_t rballoc:9;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t dai:2;
	  /// padding
	  uint32_t padding:5;
} __attribute__ ((__packed__));

typedef struct DCI1A_5MHz_TDD_1_6 DCI1A_5MHz_TDD_1_6_t;
#define sizeof_DCI1A_5MHz_TDD_1_6_t 27

///  DCI Format Type 0 (10 MHz,TDD1-6, 29 bits)
struct DCI0_10MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:11;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Cyclic shift
	  uint32_t cshift:3;
	  /// DAI
	  uint32_t dai:2;
	  /// CQI Request
	  uint32_t cqi_req:1;
	  /// Padding
	  uint32_t padding:5;
} __attribute__ ((__packed__));

typedef struct DCI0_10MHz_TDD_1_6 DCI0_10MHz_TDD_1_6_t;
#define sizeof_DCI0_10MHz_TDD_1_6_t 29

/// DCI Format Type 1A (10 MHz, TDD, frame 1-6, 30 bits)
struct DCI1A_10MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	  /// RB Assignment (ceil(log2(N_RB_DL*(N_RB_DL-1)/2)) bits)
	  uint32_t rballoc:11;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t dai:2;
	  /// SRS request bit
	  uint32_t srs_req:1;
	  /// padding
	  uint32_t padding:2;
} __attribute__ ((__packed__));

typedef struct DCI1A_10MHz_TDD_1_6 DCI1A_10MHz_TDD_1_6_t;
#define sizeof_DCI1A_10MHz_TDD_1_6_t 29


///  DCI Format Type 0 (20 MHz,TDD1-6, 27 bits)
struct DCI0_20MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Cyclic shift
	  uint32_t cshift:3;
	  /// DAI
	  uint32_t dai:2;
	  /// CQI Request
	  uint32_t cqi_req:1;
	  /// Padding
	  uint32_t padding:2;
} __attribute__ ((__packed__));

typedef struct DCI0_20MHz_TDD_1_6 DCI0_20MHz_TDD_1_6_t;
#define sizeof_DCI0_20MHz_TDD_1_6_t 31

/// DCI Format Type 1A (20 MHz, TDD, frame 1-6, 27 bits)
struct DCI1A_20MHz_TDD_1_6 {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL*(N_RB_DL-1)/2)) bits)
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t dai:2;
	  /// SRS request bit
	  uint32_t srs_req:1;
} __attribute__ ((__packed__));

typedef struct DCI1A_20MHz_TDD_1_6 DCI1A_20MHz_TDD_1_6_t;
#define sizeof_DCI1A_20MHz_TDD_1_6_t 31

///  DCI Format Type 0 (1.5 MHz,FDD, 25 bits)
struct DCI0_1_5MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:5;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DRS Cyclic Shift
	  uint32_t cshift:3;
	  /// CQI Request
	  uint32_t cqi_req:1;
	  /// Padding
	  uint32_t padding:13;
} __attribute__ ((__packed__));

typedef struct DCI0_1_5MHz_FDD DCI0_1_5MHz_FDD_t;
#define sizeof_DCI0_1_5MHz_FDD_t 21

struct DCI1A_1_5MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL(N_RB_DL+1)/2)) bits)
	  uint32_t rballoc:5;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t srs_req:1;
		/// padding
	  uint32_t padding:11;
} __attribute__ ((__packed__));

typedef struct DCI1A_1_5MHz_FDD DCI1A_1_5MHz_FDD_t;
#define sizeof_DCI1A_1_5MHz_FDD_t 21

///  DCI Format Type 0 (3 MHz,FDD, 22 bits)
struct DCI0_3MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:7;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DRS Cyclic Shift
	  uint32_t cshift:3;
	  /// CQI Request
	  uint32_t cqi_req:1;
		/// Padding
	  uint32_t padding:9;
} __attribute__ ((__packed__));

typedef struct DCI0_3MHz_FDD DCI0_3MHz_FDD_t;
#define sizeof_DCI0_3MHz_FDD_t 22

/// DCI Format Type 1 (3 MHz, FDD, 23 bits)
struct DCI1_3MHz_FDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits
	  uint32_t rballoc:8;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
		/// dummy bits (not transmitted)
	  uint32_t dummy:5;

} __attribute__ ((__packed__));

typedef struct DCI1_3MHz_FDD DCI1_3MHz_FDD_t;
#define sizeof_DCI1_3MHz_FDD_t 23

struct DCI1A_3MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL(N_RB_DL+1)/2)) bits)
	  uint32_t rballoc:7;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  //uint32_t srs_req:1;
		/// padding
	  uint32_t padding:7;

} __attribute__ ((__packed__));

typedef struct DCI1A_3MHz_FDD DCI1A_3MHz_FDD_t;
#define sizeof_DCI1A_3MHz_FDD_t 22

///  DCI Format Type 0 (5 MHz,FDD, 23 bits)
struct DCI0_5MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:9;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DRS Cyclic Shift
	  uint32_t cshift:3;
	  /// CQI Request
	  uint32_t cqi_req:1;
		/// Padding
	  uint32_t padding:9;
} __attribute__ ((__packed__));

typedef struct DCI0_5MHz_FDD DCI0_5MHz_FDD_t;
#define sizeof_DCI0_5MHz_FDD_t 25

struct DCI1A_5MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL(N_RB_DL+1)/2)) bits)
	  uint32_t rballoc:9;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t srs_req:1;
		/// padding
	  uint32_t padding:7;

} __attribute__ ((__packed__));

typedef struct DCI1A_5MHz_FDD DCI1A_5MHz_FDD_t;
#define sizeof_DCI1A_5MHz_FDD_t 25

///  DCI Format Type 0 (10 MHz,FDD, 25 bits)
struct DCI0_10MHz_FDD {
   /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
   uint32_t type:1;
   /// Hopping flag
   uint32_t hopping:1;
   /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
   uint32_t rballoc:11;
   /// Modulation and Coding Scheme and Redundancy Version
   uint32_t mcs:5;
   /// New Data Indicator
   uint32_t ndi:1;
   /// Power Control
   uint32_t TPC:2;
   /// DRS Cyclic Shift
   uint32_t cshift:3;
   /// CQI Request
   uint32_t cqi_req:1;
	/// Padding
  uint32_t padding:7;
} __attribute__ ((__packed__));

typedef struct DCI0_10MHz_FDD DCI0_10MHz_FDD_t;
#define sizeof_DCI0_10MHz_FDD_t 27

struct DCI1A_10MHz_FDD {
  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
  uint32_t type:1;
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL(N_RB_DL+1)/2)) bits)
  uint32_t rballoc:11;
  /// Modulation and Coding Scheme and Redundancy Version
   uint32_t mcs:5;
   /// HARQ Process
   uint32_t harq_pid:3;
   /// New Data Indicator
   uint32_t ndi:1;
   /// Redundancy version
   uint32_t rv:2;
   /// Power Control
   uint32_t TPC:2;
   /// Downlink Assignment Index
   uint32_t srs_req:1;
 
  /// padding
  uint32_t padding:5;
} __attribute__ ((__packed__));

typedef struct DCI1A_10MHz_FDD DCI1A_10MHz_FDD_t;
#define sizeof_DCI1A_10MHz_FDD_t 27

///  DCI Format Type 0 (20 MHz,FDD, 25 bits)
struct DCI0_20MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Hopping flag
	  uint32_t hopping:1;
	  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DRS Cyclic Shift
	  uint32_t cshift:3;
	  /// CQI Request
	  uint32_t cqi_req:1;
		/// Padding
	  uint32_t padding:5;

} __attribute__ ((__packed__));

typedef struct DCI0_20MHz_FDD DCI0_20MHz_FDD_t;
#define sizeof_DCI0_20MHz_FDD_t 28

struct DCI1A_20MHz_FDD {
	  /// type = 0 => DCI Format 0, type = 1 => DCI Format 1A 
	  uint32_t type:1;
	  /// Localized/Distributed VRB
	  uint32_t vrb_type:1;
	   /// RB Assignment (ceil(log2(N_RB_DL*(N_RB_DL+1)/2)) bits)
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// Downlink Assignment Index
	  uint32_t srs_req:1;
		/// padding
	  uint32_t padding:3;

} __attribute__ ((__packed__));

typedef struct DCI1A_20MHz_FDD DCI1A_20MHz_FDD_t;
#define sizeof_DCI1A_20MHz_FDD_t 28

/// DCI Format Type 1 (1.5 MHz, TDD, 23 bits)
struct DCI1_1_5MHz_TDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
	  uint32_t rballoc:6;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DAI (TDD)
	  uint32_t dai:2;
		/// Dummy bits to align to 32-bits
	  uint32_t dummy:9;

} __attribute__ ((__packed__));

typedef struct DCI1_1_5MHz_TDD DCI1_1_5MHz_TDD_t;
#define sizeof_DCI1_1_5MHz_TDD_t 23

/// DCI Format Type 1 (5 MHz, TDD, 30 bits)
struct DCI1_5MHz_TDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DAI (TDD)
	  uint32_t dai:2;
		/// Dummy bits to align to 32-bits
	  uint32_t dummy:2;

} __attribute__ ((__packed__));

typedef struct DCI1_5MHz_TDD DCI1_5MHz_TDD_t;
#define sizeof_DCI1_5MHz_TDD_t 30

/// DCI Format Type 1 (10 MHz, TDD, 34 bits)
struct DCI1_10MHz_TDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
	  uint32_t rballoc:17;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DAI (TDD)
	  uint32_t dai:2;
		/// Dummy bits to align to 64-bits
	  uint32_t dummy:30;

} __attribute__ ((__packed__));

typedef struct DCI1_10MHz_TDD DCI1_10MHz_TDD_t;
#define sizeof_DCI1_10MHz_TDD_t 34

/// DCI Format Type 1 (20 MHz, TDD, 42 bits)
struct DCI1_20MHz_TDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
	  uint32_t rballoc:25;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
	  /// DAI (TDD)
	  uint32_t dai:2;

	/// Dummy bits to align to 64-bits
  uint32_t dummy:22;
} __attribute__ ((__packed__));

typedef struct DCI1_20MHz_TDD DCI1_20MHz_TDD_t;
#define sizeof_DCI1_20MHz_TDD_t 42

/// DCI Format Type 1 (1.5 MHz, FDD, 21 bits)
struct DCI1_1_5MHz_FDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(N_RB_DL/P)) bits)
	  uint32_t rballoc:6;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:4;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
		/// Dummy bits to align to 32-bits
	  uint32_t dummy:11;

} __attribute__ ((__packed__));

typedef struct DCI1_1_5MHz_FDD DCI1_1_5MHz_FDD_t;
#define sizeof_DCI1_1_5MHz_FDD_t 23

/// DCI Format Type 1 (5 MHz, FDD, 27 bits)
struct DCI1_5MHz_FDD {
	  /// Resource Allocation Header
	  uint32_t rah:1;
	  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits
	  uint32_t rballoc:13;
	  /// Modulation and Coding Scheme and Redundancy Version
	  uint32_t mcs:5;
	  /// HARQ Process
	  uint32_t harq_pid:3;
	  /// New Data Indicator
	  uint32_t ndi:1;
	  /// Redundancy version
	  uint32_t rv:2;
	  /// Power Control
	  uint32_t TPC:2;
		/// dummy bits (not transmitted)
	  uint32_t dummy:5;

} __attribute__ ((__packed__));

typedef struct DCI1_5MHz_FDD DCI1_5MHz_FDD_t;
#define sizeof_DCI1_5MHz_FDD_t 27

/// DCI Format Type 1 (10 MHz, FDD, 31 bits)
struct DCI1_10MHz_FDD
{
	/// Resource Allocation Header
	uint32_t rah :1;
	/// RB Assignment (ceil(log2(N_RB_DL/P)) bits
	uint32_t rballoc :17;
	/// Modulation and Coding Scheme and Redundancy Version
	uint32_t mcs :5;
	/// HARQ Process
	uint32_t harq_pid :3;
	/// New Data Indicator
	uint32_t ndi :1;
	/// Redundancy version
	uint32_t rv :2;
	/// Power Control
	uint32_t TPC :2;
	/// dummy bits (not transmitted)
	uint32_t dummy :1;
}__attribute__ ((__packed__));

typedef struct DCI1_10MHz_FDD DCI1_10MHz_FDD_t;
#define sizeof_DCI1_10MHz_FDD_t 31

/// DCI Format Type 1B (5 MHz, FDD, 2 Antenna Ports, 27 bits)
struct DCI1B_5MHz_2A_FDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// TPMI information for precoding
  uint32_t tpmi:2;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
  /// Padding to remove size ambiguity (26 bits -> 27 bits)
  uint32_t padding:1;
} __attribute__ ((__packed__));

typedef struct DCI1B_5MHz_2A_FDD DCI1B_5MHz_2A_FDD_t;
#define sizeof_DCI1B_5MHz_FDD_t 27

/// DCI Format Type 1B (5 MHz, FDD, 4 Antenna Ports, 28 bits)
struct DCI1B_5MHz_4A_FDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// TPMI information for precoding
  uint32_t tpmi:4;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
} __attribute__ ((__packed__));

typedef struct DCI1B_5MHz_4A_FDD DCI1B_5MHz_4A_FDD_t;
#define sizeof_DCI1B_5MHz_4A_FDD_t 28

/// DCI Format Type 1B (5 MHz, TDD, 4 Antenna Ports, 31 bits)
struct DCI1B_5MHz_4A_TDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:4;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// Downlink Assignment Index
  uint32_t dai:2;
  /// TPMI information for precoding
  uint32_t tpmi:4;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
} __attribute__ ((__packed__));

typedef struct DCI1B_5MHz_4A_TDD DCI1B_5MHz_4A_TDD_t;
#define sizeof_DCI1B_5MHz_4A_TDD_t 31

/// DCI Format Type 1C (5 MHz, 12 bits)
typedef struct __attribute__ ((__packed__)){
  uint32_t rballoc:7;
  uint32_t tbs_index:5;
} DCI1C_5MHz_t;
#define sizeof_DCI1C_5MHz_t 12

/// DCI Format Type 1D (5 MHz, FDD, 2 Antenna Ports, 27 bits)
struct DCI1D_5MHz_2A_FDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// TPMI information for precoding
  uint32_t tpmi:2;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
  /// Downlink Power Offset
  uint32_t dl_power_off:1;
} __attribute__ ((__packed__));

typedef struct DCI1D_5MHz_2A_FDD DCI1D_5MHz_2A_FDD_t;
#define sizeof_DCI1D_5MHz_2A_FDD_t 27

/// DCI Format Type 1D (5 MHz, TDD, 2 Antenna Ports, 30 bits)
struct DCI1D_5MHz_2A_TDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:4;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// Downlink Assignment Index
  uint32_t dai:2;
  /// TPMI information for precoding
  uint32_t tpmi:2;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
  /// Downlink Power Offset
  uint32_t dl_power_off:1;
} __attribute__ ((__packed__));

typedef struct DCI1D_5MHz_2A_TDD DCI1D_5MHz_2A_TDD_t;
#define sizeof_DCI1D_5MHz_2A_TDD_t 30

/// DCI Format Type 1D (5 MHz, FDD, 4 Antenna Ports, 29 bits)
struct DCI1D_5MHz_4A_FDD {
  /// Localized/Distributed VRB
  uint32_t vrb_type:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:9;
  /// Modulation and Coding Scheme and Redundancy Version
  uint32_t mcs:5;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// New Data Indicator
  uint32_t ndi:1;
  /// Redundancy version
  uint32_t rv:2;
  /// Power Control
  uint32_t TPC:2;
  /// TPMI information for precoding
  uint32_t tpmi:4;
  /// TMI confirmation for precoding
  uint32_t pmi:1;
  /// Downlink Power Offset
  uint32_t dl_power_off:1;
}  __attribute__ ((__packed__));

typedef struct DCI1D_5MHz_4A_FDD DCI1D_5MHz_4A_FDD_t;
#define sizeof_DCI1D_5MHz_4A_FDD_t 29

/// DCI Format Type 2 (5 MHz, FDD, 2 Antenna Ports, less than 10 PRBs, 38 bits)
struct DCI2_5MHz_2A_L10PRB_FDD {
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:3;
} __attribute__ ((__packed__));

typedef struct DCI2_5MHz_2A_L10PRB_FDD DCI2_5MHz_2A_L10PRB_FDD_t;
#define sizeof_DCI2_5MHz_2A_L10PRB_FDD_t 38

/// DCI Format Type 2 (5 MHz, FDD, 4 Antenna Ports, less than 10 PRBs, 41 bits)
typedef struct __attribute__ ((__packed__)){
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:6;
} DCI2_5MHz_4A_L10PRB_FDD_t;
#define sizeof_DCI2_5MHz_4A_L10PRB_FDD_t 41

/// DCI Format Type 2 (5 MHz, FDD, 2 Antenna Ports, more than 10 PRBs, 39 bits)
typedef struct __attribute__ ((__packed__)){
  /// Resource Allocation Header
  uint32_t rah:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:3;
} DCI2_5MHz_2A_M10PRB_FDD_t;
#define sizeof_DCI2_5MHz_2A_M10PRB_FDD_t 39

/// DCI Format Type 2 (5 MHz, TDD, 4 Antenna Ports, more than 10 PRBs, 42 bits)
typedef struct __attribute__ ((__packed__)){
  /// Resource Allocation Header
  uint32_t rah:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:6;
} DCI2_5MHz_4A_M10PRB_FDD_t;
#define sizeof_DCI2_5MHz_4A_M10PRB_FDD_t 42

/// DCI Format Type 2A (5 MHz, FDD, 2 Antenna Ports, less than 10 PRBs, 35 bits)
typedef struct __attribute__ ((__packed__)){
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
} DCI2A_5MHz_2A_L10PRB_FDD_t;
#define sizeof_DCI2A_5MHz_2A_L10PRB_FDD_t 35

/// DCI Format Type 2A (5 MHz, FDD, 4 Antenna Ports, less than 10 PRBs, 37 bits)
typedef struct __attribute__ ((__packed__)){
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:2;
} DCI2A_5MHz_4A_L10PRB_FDD_t;
#define sizeof_DCI2A_5MHz_4A_L10PRB_FDD_t 37

/// DCI Format Type 2A (5 MHz, FDD, 2 Antenna Ports, more than 10 PRBs, 36 bits)
typedef struct __attribute__ ((__packed__)){
  /// Resource Allocation Header
  uint32_t rah:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
} DCI2A_5MHz_2A_M10PRB_FDD_t;
#define sizeof_DCI2A_5MHz_2A_M10PRB_FDD_t 36

/// DCI Format Type 2A (5 MHz, TDD, 4 Antenna Ports, more than 10 PRBs, 38 bits)
typedef struct __attribute__ ((__packed__)){
  /// Resource Allocation Header
  uint32_t rah:1;
  /// RB Assignment (ceil(log2(N_RB_DL/P)) bits)
  uint32_t rballoc:13;
  /// Power Control
  uint32_t TPC:2;
  /// HARQ Process
  uint32_t harq_pid:3;
  /// TB swap
  uint32_t tb_swap:1;
  /// Modulation and Coding Scheme and Redundancy Version 1
  uint32_t mcs1:5;
  /// New Data Indicator 1
  uint32_t ndi1:1;
  /// Redundancy version 1
  uint32_t rv1:2;
  /// Modulation and Coding Scheme and Redundancy Version 2
  uint32_t mcs2:5;
  /// New Data Indicator 2
  uint32_t ndi2:1;
  /// Redundancy version 2
  uint32_t rv2:2;
  /// TPMI information for precoding
  uint32_t tpmi:2;
} DCI2A_5MHz_4A_M10PRB_FDD_t;
#define sizeof_DCI2A_5MHz_4A_M10PRB_FDD_t 38

typedef struct __attribute__ ((__packed__)){
  uint32_t TPC:26;
} DCI3_5MHz_FDD_t;
#define sizeof_DCI3_5MHz_FDD_t 25

///  DCI Format Type 0 (1.5 MHz,9 bits)
struct DCI0A_1_5MHz {
  /// Padding
  uint32_t padding:19;
    /// Cyclic shift
  uint32_t cshift:3;
  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
  uint32_t rballoc:5;
  /// Hopping flag
  uint32_t hopping:1;
} __attribute__ ((__packed__));
#define sizeof_DCI0A_1_5MHz 9

///  DCI Format Type 0 (5 MHz,13 bits)
struct DCI0A_5MHz {
  /// Padding
  uint32_t padding:19;
    /// Cyclic shift
  uint32_t cshift:3;
  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
  uint32_t rballoc:9;
  /// Hopping flag
  uint32_t hopping:1;
} __attribute__ ((__packed__));
#define sizeof_DCI0A_5MHz 13

///  DCI Format Type 0 (10 MHz,15 bits)
struct DCI0A_10_MHz {
  /// Padding
  uint32_t padding:17;
    /// Cyclic shift
  uint32_t cshift:3;
  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
  uint32_t rballoc:11;
  /// Hopping flag
  uint32_t hopping:1;
} __attribute__ ((__packed__));
#define sizeof_DCI0A_10MHz 15

///  DCI Format Type 0 (20 MHz,17 bits)
struct DCI0A_20_MHz {
  /// Padding
  uint32_t padding:15;
    /// Cyclic shift
  uint32_t cshift:3;
  /// RB Assignment (ceil(log2(N_RB_UL*(N_RB_UL+1)/2)) bits)
  uint32_t rballoc:13;
  /// Hopping flag
  uint32_t hopping:1;
} __attribute__ ((__packed__));
#define sizeof_DCI0A_20MHz 17

#define MAX_DCI_SIZE_BITS 45

#endif
