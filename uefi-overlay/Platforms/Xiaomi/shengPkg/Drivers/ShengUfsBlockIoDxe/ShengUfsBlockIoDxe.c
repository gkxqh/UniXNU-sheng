// SPDX-License-Identifier: BSD-2-Clause-Patent
// Polled UFS LUN 0 EFI_BLOCK_IO_PROTOCOL driver for Xiaomi Pad 6S Pro (sheng / SM8550).
// Reuses the ABL-initialized UFS host controller (0x01D84000) and SMMU stream 0x60 S1 bypass.

#include <Uefi.h>
#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define UFS_HC_BASE             0x01D84000ULL
#define APPS_SMMU_BASE          0x15000000ULL
#define UFS_STREAM_ID           0x0060U

#define UFS_REG_CAP             0x00U
#define UFS_REG_VER             0x08U
#define UFS_REG_IS              0x20U
#define UFS_REG_IE              0x24U
#define UFS_REG_HCS             0x30U
#define UFS_REG_UTRLBA          0x50U
#define UFS_REG_UTRLBAU         0x54U
#define UFS_REG_UTRLDBR         0x58U
#define UFS_REG_UTRLCLR         0x5CU
#define UFS_REG_UTRLRSR         0x60U
#define UFS_REG_UTRLCNR         0x68U

#define SMMU_IDR0               0x0020U
#define SMMU_IDR1               0x0024U
#define SMMU_SMR_BASE           0x0800U
#define SMMU_S2CR_BASE          0x0C00U
#define SMMU_CBAR_BASE          0x1000U
#define SMMU_CBA2R_BASE         0x1800U

#define UFS_BLOCK_SIZE          4096U
#define UFS_MAX_BLOCKS_PER_CMD  64U
#define UFS_DATA_BUFFER_BYTES   (UFS_BLOCK_SIZE * UFS_MAX_BLOCKS_PER_CMD)
#define UFS_DMA_PAGES           66U

#define UFS_UTRD_OFFSET         0x0000U
#define UFS_UCD_OFFSET          0x1000U
#define UFS_RSP_OFFSET          0x1200U
#define UFS_PRDT_OFFSET         0x1400U
#define UFS_DATA_OFFSET         0x2000U

// Userdata partition bounds on sheng UFS LUN 0 (for safe write gating)
#define SHENG_USERDATA_FIRST_LBA  3094592ULL
#define SHENG_USERDATA_LAST_LBA   64007162ULL

#pragma pack(1)
typedef struct {
  VENDOR_DEVICE_PATH       UfsLun0;
  EFI_DEVICE_PATH_PROTOCOL End;
} SHENG_UFS_DEVICE_PATH;
#pragma pack()

typedef struct {
  EFI_HANDLE               Handle;
  EFI_BLOCK_IO_PROTOCOL    BlockIo;
  EFI_BLOCK_IO_MEDIA       Media;
  EFI_PHYSICAL_ADDRESS     DmaPhys;
  UINT8                    *DmaVirt;
  UINT8                    TaskTag;
} SHENG_UFS_DEV;

STATIC SHENG_UFS_DEV mShengUfsDev;

STATIC SHENG_UFS_DEVICE_PATH mShengUfsDevicePath = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
        (UINT8)(sizeof (VENDOR_DEVICE_PATH)),
        (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8)
      }
    },
    { 0x860845C1, 0xBE09, 0x4355, { 0x8B, 0xC1, 0x30, 0xD6, 0x4F, 0xF8, 0xE6, 0x3A } }
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      (UINT8)(sizeof (EFI_DEVICE_PATH_PROTOCOL)),
      (UINT8)((sizeof (EFI_DEVICE_PATH_PROTOCOL)) >> 8)
    }
  }
};

STATIC
UINT32
ReadBe32 (
  IN CONST UINT8 *Buf
  )
{
  return ((UINT32)Buf[0] << 24) |
         ((UINT32)Buf[1] << 16) |
         ((UINT32)Buf[2] << 8)  |
         (UINT32)Buf[3];
}

STATIC
VOID
WriteBe32 (
  OUT UINT8  *Buf,
  IN  UINT32 Val
  )
{
  Buf[0] = (UINT8)((Val >> 24) & 0xFFU);
  Buf[1] = (UINT8)((Val >> 16) & 0xFFU);
  Buf[2] = (UINT8)((Val >> 8) & 0xFFU);
  Buf[3] = (UINT8)(Val & 0xFFU);
}

STATIC
VOID
WriteLe32 (
  OUT UINT8  *Buf,
  IN  UINT32 Val
  )
{
  Buf[0] = (UINT8)(Val & 0xFFU);
  Buf[1] = (UINT8)((Val >> 8) & 0xFFU);
  Buf[2] = (UINT8)((Val >> 16) & 0xFFU);
  Buf[3] = (UINT8)((Val >> 24) & 0xFFU);
}

STATIC
EFI_STATUS
ConfigureUfsSmmuBypass (
  VOID
  )
{
  UINT32 Idr0;
  UINT32 Idr1;
  UINT32 StreamMatchCount;
  UINT32 ContextBankCount;
  UINT32 Pages;
  UINT32 Index;
  INT32  MatchedSlot;
  INT32  FreeSlot;
  INT32  ChosenBank;

  Idr0 = MmioRead32 (APPS_SMMU_BASE + SMMU_IDR0);
  Idr1 = MmioRead32 (APPS_SMMU_BASE + SMMU_IDR1);

  StreamMatchCount = Idr0 & 0xFFU;
  ContextBankCount = Idr1 & 0xFFU;
  if (StreamMatchCount > 128U) {
    StreamMatchCount = 128U;
  }
  if (ContextBankCount > 128U) {
    ContextBankCount = 128U;
  }
  if ((StreamMatchCount == 0U) || (ContextBankCount == 0U)) {
    return EFI_DEVICE_ERROR;
  }

  Pages = ((Idr1 & (1U << 31)) != 0U) ? 0x100U : 0x10U;
  MatchedSlot = -1;
  FreeSlot    = -1;

  for (Index = 0; Index < StreamMatchCount; Index++) {
    UINT32 Smr  = MmioRead32 (APPS_SMMU_BASE + SMMU_SMR_BASE + (Index * 4U));
    BOOLEAN Valid = ((Smr & 0x80000000U) != 0U);
    UINT16 Id   = (UINT16)(Smr & 0x7FFFU);
    UINT16 Mask = (UINT16)((Smr >> 16) & 0x7FFFU);

    if (!Valid && (FreeSlot < 0)) {
      FreeSlot = (INT32)Index;
    }
    if (Valid && ((((Id ^ UFS_STREAM_ID) & (UINT16)(~Mask)) & 0x7FFFU) == 0U)) {
      MatchedSlot = (INT32)Index;
      break;
    }
  }

  if (MatchedSlot >= 0) {
    UINT32 S2cr = MmioRead32 (APPS_SMMU_BASE + SMMU_S2CR_BASE + ((UINT32)MatchedSlot * 4U));
    UINT32 Type = (S2cr >> 16) & 0x3U;
    UINT32 Bank = S2cr & 0xFFU;
    if ((Type == 0U) && (Bank < ContextBankCount)) {
      UINT32 Cbar  = MmioRead32 (APPS_SMMU_BASE + SMMU_CBAR_BASE + (Bank * 4U));
      UINT32 Sctlr = MmioRead32 (APPS_SMMU_BASE + ((Pages + Bank) * 0x1000U));
      if ((Cbar == 0x00010000U) && ((Sctlr & 0x1U) == 0U)) {
        return EFI_SUCCESS;
      }
    }
    FreeSlot = MatchedSlot;
  }

  if (FreeSlot < 0) {
    return EFI_OUT_OF_RESOURCES;
  }

  ChosenBank = -1;
  for (Index = 0; Index < ContextBankCount; Index++) {
    UINT32 Candidate = ContextBankCount - 1U - Index;
    BOOLEAN InUse = FALSE;
    UINT32  SmrIdx;
    for (SmrIdx = 0; SmrIdx < StreamMatchCount; SmrIdx++) {
      UINT32 Smr  = MmioRead32 (APPS_SMMU_BASE + SMMU_SMR_BASE + (SmrIdx * 4U));
      UINT32 S2cr = MmioRead32 (APPS_SMMU_BASE + SMMU_S2CR_BASE + (SmrIdx * 4U));
      if (((Smr & 0x80000000U) != 0U) && (((S2cr >> 16) & 0x3U) == 0U) && ((S2cr & 0xFFU) == Candidate)) {
        InUse = TRUE;
        break;
      }
    }
    if (!InUse) {
      ChosenBank = (INT32)Candidate;
      break;
    }
  }

  if (ChosenBank < 0) {
    return EFI_OUT_OF_RESOURCES;
  }

  MmioWrite32 (APPS_SMMU_BASE + ((Pages + (UINT32)ChosenBank) * 0x1000U), 0U);
  MmioWrite32 (APPS_SMMU_BASE + SMMU_CBA2R_BASE + ((UINT32)ChosenBank * 4U), 0x1U);
  MmioWrite32 (APPS_SMMU_BASE + SMMU_CBAR_BASE + ((UINT32)ChosenBank * 4U), 0x00010000U);
  MmioWrite32 (APPS_SMMU_BASE + SMMU_S2CR_BASE + ((UINT32)FreeSlot * 4U), (UINT32)ChosenBank & 0xFFU);
  MmioWrite32 (APPS_SMMU_BASE + SMMU_SMR_BASE + ((UINT32)FreeSlot * 4U), 0x80000000U | (UFS_STREAM_ID & 0x7FFFU));
  MemoryFence ();

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
StopUfsTransferQueue (
  VOID
  )
{
  UINT32 Poll;

  MmioWrite32 (UFS_HC_BASE + UFS_REG_IE, 0U);
  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLRSR, 0U);
  MemoryFence ();

  for (Poll = 0; Poll < 4000U; Poll++) {
    if ((MmioRead32 (UFS_HC_BASE + UFS_REG_UTRLRSR) & 0x1U) == 0U) {
      return EFI_SUCCESS;
    }
    gBS->Stall (10);
  }

  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
StartUfsTransferQueue (
  IN EFI_PHYSICAL_ADDRESS UtrlDmaAddr
  )
{
  UINT32 Poll;
  UINT32 IsVal;

  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLBA, (UINT32)(UtrlDmaAddr & 0xFFFFFFFFULL));
  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLBAU, (UINT32)(UtrlDmaAddr >> 32));
  IsVal = MmioRead32 (UFS_HC_BASE + UFS_REG_IS);
  if (IsVal != 0U) {
    MmioWrite32 (UFS_HC_BASE + UFS_REG_IS, IsVal);
  }
  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLCNR, 0x1U);
  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLRSR, 0x1U);
  MemoryFence ();

  for (Poll = 0; Poll < 4000U; Poll++) {
    UINT32 UtRlRsr = MmioRead32 (UFS_HC_BASE + UFS_REG_UTRLRSR);
    UINT32 Hcs     = MmioRead32 (UFS_HC_BASE + UFS_REG_HCS);
    if (((UtRlRsr & 0x1U) == 0x1U) && ((Hcs & 0x3U) == 0x3U)) {
      return EFI_SUCCESS;
    }
    gBS->Stall (10);
  }

  return EFI_TIMEOUT;
}

STATIC
VOID
PrepareSlot0Utrd (
  OUT UINT8                *Utrd,
  IN  UINT32               DataDirection,
  IN  EFI_PHYSICAL_ADDRESS UcdDmaAddr,
  IN  UINT16               PrdtLengthEntries
  )
{
  ZeroMem (Utrd, 32);
  WriteLe32 (Utrd + 0x00, ((0x1U << 28) | ((DataDirection & 0x3U) << 25) | (1U << 24)));
  WriteLe32 (Utrd + 0x08, 0x0000000FU);
  WriteLe32 (Utrd + 0x10, (UINT32)(UcdDmaAddr & 0xFFFFFF80ULL));
  WriteLe32 (Utrd + 0x14, (UINT32)(UcdDmaAddr >> 32));
  WriteLe32 (Utrd + 0x18, (0x0080U << 16) | 0x0080U);
  WriteLe32 (Utrd + 0x1C, (0x0100U << 16) | (UINT32)PrdtLengthEntries);
}

STATIC
EFI_STATUS
SubmitSlot0AndPoll (
  IN  SHENG_UFS_DEV *Dev,
  IN  UINTN         FlushDataBytes,
  IN  BOOLEAN       InvalidateDataAfter,
  OUT UINT8         *OcsOut OPTIONAL,
  OUT UINT8         *ResponseStatusOut OPTIONAL
  )
{
  UINT32 IsVal;
  UINT32 Poll;
  UINT8  *Utrd;
  UINT8  *Rsp;

  Utrd = Dev->DmaVirt + UFS_UTRD_OFFSET;
  Rsp  = Dev->DmaVirt + UFS_RSP_OFFSET;

  WriteBackInvalidateDataCacheRange (Dev->DmaVirt, UFS_DATA_OFFSET + FlushDataBytes);
  MemoryFence ();

  IsVal = MmioRead32 (UFS_HC_BASE + UFS_REG_IS);
  if (IsVal != 0U) {
    MmioWrite32 (UFS_HC_BASE + UFS_REG_IS, IsVal);
  }

  if ((MmioRead32 (UFS_HC_BASE + UFS_REG_UTRLDBR) & 0x1U) != 0U) {
    MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLCLR, 0xFFFFFFFEU);
    MemoryFence ();
  }

  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLDBR, 0x1U);
  MemoryFence ();

  for (Poll = 0; Poll < 500000U; Poll++) {
    UINT32 Dbr = MmioRead32 (UFS_HC_BASE + UFS_REG_UTRLDBR);
    if ((Dbr & 0x1U) == 0U) {
      UINT32 Cnr = MmioRead32 (UFS_HC_BASE + UFS_REG_UTRLCNR);
      if ((Cnr & 0x1U) != 0U) {
        MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLCNR, 0x1U);
      }
      InvalidateDataCacheRange (Dev->DmaVirt, UFS_DATA_OFFSET + (InvalidateDataAfter ? FlushDataBytes : 0U));
      MemoryFence ();

      if (OcsOut != NULL) {
        *OcsOut = Utrd[8];
      }
      if (ResponseStatusOut != NULL) {
        *ResponseStatusOut = Rsp[3];
      }
      if (Utrd[8] != 0U) {
        return EFI_DEVICE_ERROR;
      }
      return EFI_SUCCESS;
    }
    gBS->Stall (2);
  }

  MmioWrite32 (UFS_HC_BASE + UFS_REG_UTRLCLR, 0xFFFFFFFEU);
  MemoryFence ();
  return EFI_TIMEOUT;
}

STATIC
UINT8
NextTaskTag (
  IN OUT SHENG_UFS_DEV *Dev
  )
{
  Dev->TaskTag++;
  if ((Dev->TaskTag == 0U) || (Dev->TaskTag >= 32U)) {
    Dev->TaskTag = 1U;
  }
  return Dev->TaskTag;
}

STATIC
EFI_STATUS
SendUfsNopOut (
  IN OUT SHENG_UFS_DEV *Dev
  )
{
  UINT8      *Utrd;
  UINT8      *Ucd;
  UINT8      *Rsp;
  EFI_STATUS Status;

  Utrd = Dev->DmaVirt + UFS_UTRD_OFFSET;
  Ucd  = Dev->DmaVirt + UFS_UCD_OFFSET;
  Rsp  = Dev->DmaVirt + UFS_RSP_OFFSET;

  ZeroMem (Ucd, 0x500);
  Ucd[0] = 0x00U;
  Ucd[3] = NextTaskTag (Dev);

  PrepareSlot0Utrd (Utrd, 0U, Dev->DmaPhys + UFS_UCD_OFFSET, 0U);
  Status = SubmitSlot0AndPoll (Dev, 0U, FALSE, NULL, NULL);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return ((Rsp[0] & 0x3FU) == 0x20U) ? EFI_SUCCESS : EFI_DEVICE_ERROR;
}

STATIC
EFI_STATUS
SendUfsQueryReadAttr (
  IN OUT SHENG_UFS_DEV *Dev,
  IN     UINT8         Idn,
  OUT    UINT32        *AttrValueOut
  )
{
  UINT8      *Utrd;
  UINT8      *Ucd;
  UINT8      *Rsp;
  UINT8      RespStatus;
  EFI_STATUS Status;

  Utrd = Dev->DmaVirt + UFS_UTRD_OFFSET;
  Ucd  = Dev->DmaVirt + UFS_UCD_OFFSET;
  Rsp  = Dev->DmaVirt + UFS_RSP_OFFSET;

  ZeroMem (Ucd, 0x500);
  Ucd[0]  = 0x16U;
  Ucd[3]  = NextTaskTag (Dev);
  Ucd[5]  = 0x01U;
  Ucd[12] = 0x03U;
  Ucd[13] = Idn;

  PrepareSlot0Utrd (Utrd, 0U, Dev->DmaPhys + UFS_UCD_OFFSET, 0U);
  Status = SubmitSlot0AndPoll (Dev, 0U, FALSE, NULL, &RespStatus);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  if (((Rsp[0] & 0x3FU) != 0x36U) || (RespStatus != 0U)) {
    return EFI_DEVICE_ERROR;
  }

  *AttrValueOut = ReadBe32 (Rsp + 20);
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SendUfsScsiCmd (
  IN OUT SHENG_UFS_DEV *Dev,
  IN     UINT8         Lun,
  IN     CONST UINT8   *Cdb,
  IN     UINTN         CdbLen,
  IN     UINT32        DataDirection,
  IN     UINT32        TransferBytes
  )
{
  UINT8      *Utrd;
  UINT8      *Ucd;
  UINT8      *Rsp;
  UINT8      *Prdt;
  UINT8      RespStatus;
  EFI_STATUS Status;

  Utrd = Dev->DmaVirt + UFS_UTRD_OFFSET;
  Ucd  = Dev->DmaVirt + UFS_UCD_OFFSET;
  Rsp  = Dev->DmaVirt + UFS_RSP_OFFSET;
  Prdt = Dev->DmaVirt + UFS_PRDT_OFFSET;

  ZeroMem (Ucd, 0x500);
  Ucd[0] = 0x01U;
  if (DataDirection == 2U) {
    Ucd[1] = 0x40U;
  } else if (DataDirection == 1U) {
    Ucd[1] = 0x20U;
  } else {
    Ucd[1] = 0x00U;
  }
  Ucd[2] = Lun;
  Ucd[3] = NextTaskTag (Dev);
  WriteBe32 (Ucd + 12, TransferBytes);
  CopyMem (Ucd + 16, Cdb, CdbLen);

  if (TransferBytes > 0U) {
    EFI_PHYSICAL_ADDRESS DataPhys = Dev->DmaPhys + UFS_DATA_OFFSET;
    WriteLe32 (Prdt + 0x00, (UINT32)(DataPhys & 0xFFFFFFFCULL));
    WriteLe32 (Prdt + 0x04, (UINT32)(DataPhys >> 32));
    WriteLe32 (Prdt + 0x08, 0U);
    WriteLe32 (Prdt + 0x0C, TransferBytes - 1U);
    PrepareSlot0Utrd (Utrd, DataDirection, Dev->DmaPhys + UFS_UCD_OFFSET, 1U);
  } else {
    PrepareSlot0Utrd (Utrd, 0U, Dev->DmaPhys + UFS_UCD_OFFSET, 0U);
  }

  Status = SubmitSlot0AndPoll (
             Dev,
             TransferBytes,
             (DataDirection == 2U),
             NULL,
             &RespStatus
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  if (((Rsp[0] & 0x3FU) != 0x21U) || (RespStatus != 0U)) {
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
BringUpShengUfsLun0 (
  IN OUT SHENG_UFS_DEV *Dev,
  OUT    EFI_LBA       *LastBlockOut
  )
{
  UINT32     Cap;
  UINT32     Ver;
  UINT32     PowerMode;
  UINT32     Attempt;
  EFI_STATUS Status;
  UINT8      TurCdb[6]      = { 0x00U, 0U, 0U, 0U, 0U, 0U };
  UINT8      StartStopCdb[6] = { 0x1BU, 0U, 0U, 0U, 0x10U, 0U };
  UINT8      ReadCap10Cdb[10] = { 0x25U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

  Cap = MmioRead32 (UFS_HC_BASE + UFS_REG_CAP);
  Ver = MmioRead32 (UFS_HC_BASE + UFS_REG_VER);
  if ((Cap == 0U) || (Cap == 0xFFFFFFFFU) || (Ver == 0U) || (Ver == 0xFFFFFFFFU)) {
    return EFI_DEVICE_ERROR;
  }

  Status = ConfigureUfsSmmuBypass ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = StopUfsTransferQueue ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = StartUfsTransferQueue (Dev->DmaPhys + UFS_UTRD_OFFSET);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SendUfsNopOut (Dev);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  PowerMode = 0U;
  Status = SendUfsQueryReadAttr (Dev, 0x02U, &PowerMode);
  if (!EFI_ERROR (Status) && ((PowerMode & 0xFFU) != 0x11U)) {
    (VOID)SendUfsScsiCmd (Dev, 0xD0U, StartStopCdb, sizeof (StartStopCdb), 0U, 0U);
    gBS->Stall (50000);
  }

  for (Attempt = 0; Attempt < 25U; Attempt++) {
    Status = SendUfsScsiCmd (Dev, 0U, TurCdb, sizeof (TurCdb), 0U, 0U);
    if (!EFI_ERROR (Status)) {
      break;
    }
    gBS->Stall (10000);
  }
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SendUfsScsiCmd (Dev, 0U, ReadCap10Cdb, sizeof (ReadCap10Cdb), 2U, 8U);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  *LastBlockOut = (EFI_LBA)ReadBe32 (Dev->DmaVirt + UFS_DATA_OFFSET);
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShengUfsReset (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN BOOLEAN               ExtendedVerification
  )
{
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShengUfsReadBlocks (
  IN  EFI_BLOCK_IO_PROTOCOL *This,
  IN  UINT32                MediaId,
  IN  EFI_LBA               Lba,
  IN  UINTN                 BufferSize,
  OUT VOID                  *Buffer
  )
{
  UINTN      TotalBlocks;
  UINTN      RemainingBlocks;
  EFI_LBA    CurrentLba;
  UINT8      *Dst;
  EFI_STATUS Status;

  if (BufferSize == 0U) {
    return EFI_SUCCESS;
  }
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  if (MediaId != This->Media->MediaId) {
    return EFI_MEDIA_CHANGED;
  }
  if ((BufferSize % UFS_BLOCK_SIZE) != 0U) {
    return EFI_BAD_BUFFER_SIZE;
  }

  TotalBlocks = BufferSize / UFS_BLOCK_SIZE;
  if ((Lba > This->Media->LastBlock) || ((Lba + TotalBlocks - 1U) > This->Media->LastBlock)) {
    return EFI_INVALID_PARAMETER;
  }

  RemainingBlocks = TotalBlocks;
  CurrentLba      = Lba;
  Dst             = (UINT8 *)Buffer;

  while (RemainingBlocks > 0U) {
    UINT32 ChunkBlocks = (RemainingBlocks > UFS_MAX_BLOCKS_PER_CMD)
                           ? UFS_MAX_BLOCKS_PER_CMD
                           : (UINT32)RemainingBlocks;
    UINT32 ChunkBytes  = ChunkBlocks * UFS_BLOCK_SIZE;
    UINT8  Read10Cdb[10];

    ZeroMem (Read10Cdb, sizeof (Read10Cdb));
    Read10Cdb[0] = 0x28U;
    WriteBe32 (Read10Cdb + 2, (UINT32)CurrentLba);
    Read10Cdb[7] = (UINT8)((ChunkBlocks >> 8) & 0xFFU);
    Read10Cdb[8] = (UINT8)(ChunkBlocks & 0xFFU);

    Status = SendUfsScsiCmd (&mShengUfsDev, 0U, Read10Cdb, sizeof (Read10Cdb), 2U, ChunkBytes);
    if (EFI_ERROR (Status)) {
      return EFI_DEVICE_ERROR;
    }

    CopyMem (Dst, mShengUfsDev.DmaVirt + UFS_DATA_OFFSET, ChunkBytes);
    Dst             += ChunkBytes;
    CurrentLba      += ChunkBlocks;
    RemainingBlocks -= ChunkBlocks;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShengUfsWriteBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN UINT32                MediaId,
  IN EFI_LBA               Lba,
  IN UINTN                 BufferSize,
  IN VOID                  *Buffer
  )
{
  UINTN      TotalBlocks;
  UINTN      RemainingBlocks;
  EFI_LBA    CurrentLba;
  CONST UINT8 *Src;
  EFI_STATUS Status;

  if (BufferSize == 0U) {
    return EFI_SUCCESS;
  }
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  if (MediaId != This->Media->MediaId) {
    return EFI_MEDIA_CHANGED;
  }
  if ((BufferSize % UFS_BLOCK_SIZE) != 0U) {
    return EFI_BAD_BUFFER_SIZE;
  }

  TotalBlocks = BufferSize / UFS_BLOCK_SIZE;
  if ((Lba > This->Media->LastBlock) || ((Lba + TotalBlocks - 1U) > This->Media->LastBlock)) {
    return EFI_INVALID_PARAMETER;
  }

  // Only permit writes within the userdata partition on UFS LUN 0
  if ((Lba < SHENG_USERDATA_FIRST_LBA) || ((Lba + TotalBlocks - 1U) > SHENG_USERDATA_LAST_LBA)) {
    return EFI_WRITE_PROTECTED;
  }

  RemainingBlocks = TotalBlocks;
  CurrentLba      = Lba;
  Src             = (CONST UINT8 *)Buffer;

  while (RemainingBlocks > 0U) {
    UINT32 ChunkBlocks = (RemainingBlocks > UFS_MAX_BLOCKS_PER_CMD)
                           ? UFS_MAX_BLOCKS_PER_CMD
                           : (UINT32)RemainingBlocks;
    UINT32 ChunkBytes  = ChunkBlocks * UFS_BLOCK_SIZE;
    UINT8  Write10Cdb[10];

    CopyMem (mShengUfsDev.DmaVirt + UFS_DATA_OFFSET, Src, ChunkBytes);

    ZeroMem (Write10Cdb, sizeof (Write10Cdb));
    Write10Cdb[0] = 0x2AU;
    WriteBe32 (Write10Cdb + 2, (UINT32)CurrentLba);
    Write10Cdb[7] = (UINT8)((ChunkBlocks >> 8) & 0xFFU);
    Write10Cdb[8] = (UINT8)(ChunkBlocks & 0xFFU);

    Status = SendUfsScsiCmd (&mShengUfsDev, 0U, Write10Cdb, sizeof (Write10Cdb), 1U, ChunkBytes);
    if (EFI_ERROR (Status)) {
      return EFI_DEVICE_ERROR;
    }

    Src             += ChunkBytes;
    CurrentLba      += ChunkBlocks;
    RemainingBlocks -= ChunkBlocks;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShengUfsFlushBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This
  )
{
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShengUfsBlockIoDxeEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS           Status;
  EFI_PHYSICAL_ADDRESS DmaPages;
  EFI_LBA              LastBlock;

  ZeroMem (&mShengUfsDev, sizeof (mShengUfsDev));

  DmaPages = 0xFFFFFFFFULL;
  Status = gBS->AllocatePages (
                  AllocateMaxAddress,
                  EfiBootServicesData,
                  UFS_DMA_PAGES,
                  &DmaPages
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  mShengUfsDev.DmaPhys = DmaPages;
  mShengUfsDev.DmaVirt = (UINT8 *)(UINTN)DmaPages;
  ZeroMem (mShengUfsDev.DmaVirt, EFI_PAGES_TO_SIZE (UFS_DMA_PAGES));
  WriteBackInvalidateDataCacheRange (mShengUfsDev.DmaVirt, EFI_PAGES_TO_SIZE (UFS_DMA_PAGES));

  LastBlock = 0;
  Status = BringUpShengUfsLun0 (&mShengUfsDev, &LastBlock);
  if (EFI_ERROR (Status) || (LastBlock == 0)) {
    gBS->FreePages (DmaPages, UFS_DMA_PAGES);
    return EFI_DEVICE_ERROR;
  }

  mShengUfsDev.Media.MediaId          = 1U;
  mShengUfsDev.Media.RemovableMedia   = FALSE;
  mShengUfsDev.Media.MediaPresent     = TRUE;
  mShengUfsDev.Media.LogicalPartition = FALSE;
  mShengUfsDev.Media.ReadOnly         = FALSE;
  mShengUfsDev.Media.WriteCaching     = FALSE;
  mShengUfsDev.Media.BlockSize        = UFS_BLOCK_SIZE;
  mShengUfsDev.Media.IoAlign          = 4U;
  mShengUfsDev.Media.LastBlock        = LastBlock;

  mShengUfsDev.BlockIo.Revision    = EFI_BLOCK_IO_PROTOCOL_REVISION3;
  mShengUfsDev.BlockIo.Media       = &mShengUfsDev.Media;
  mShengUfsDev.BlockIo.Reset       = ShengUfsReset;
  mShengUfsDev.BlockIo.ReadBlocks  = ShengUfsReadBlocks;
  mShengUfsDev.BlockIo.WriteBlocks = ShengUfsWriteBlocks;
  mShengUfsDev.BlockIo.FlushBlocks = ShengUfsFlushBlocks;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &mShengUfsDev.Handle,
                  &gEfiDevicePathProtocolGuid,
                  &mShengUfsDevicePath,
                  &gEfiBlockIoProtocolGuid,
                  &mShengUfsDev.BlockIo,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    gBS->FreePages (DmaPages, UFS_DMA_PAGES);
    return Status;
  }

  (VOID)gBS->ConnectController (mShengUfsDev.Handle, NULL, NULL, TRUE);
  return EFI_SUCCESS;
}
