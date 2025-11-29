//! BLE Compatibility Layer
//!
//! Bridges bt-hci@0.3 (used by nrf-sdc) with bt-hci@0.2 (used by trouble-host@0.1.0).
//!
//! **Strategy**: Since HCI commands are binary-compatible between versions (they follow
//! the Bluetooth specification), we convert between types by serializing to bytes and
//! deserializing. The underlying `SoftdeviceController` handles all the actual BLE operations.

use microbit_bsp::ble::SoftdeviceController;
use trouble_host::Controller as TroubleController;
use embedded_io::ErrorType;

// Import bt-hci@0.2 (used by trouble-host)
use bt_hci::controller::{Controller as BtHciController, ControllerCmdSync, ControllerCmdAsync};
use bt_hci::cmd::{SyncCmd, AsyncCmd};
use bt_hci::{AsHciBytes, WriteHci, FromHciBytes, ControllerToHostPacket};
use bt_hci::data::{AclPacket, SyncPacket, IsoPacket};

// Import bt-hci@0.3 types (renamed to avoid conflicts)
use bt_hci_v3::controller::{ControllerCmdSync as ControllerCmdSyncV3, ControllerCmdAsync as ControllerCmdAsyncV3};
use bt_hci_v3::cmd::le::LeReadBufferSize as LeReadBufferSizeV3;
use bt_hci_v3::data::{AclPacket as AclPacketV3, SyncPacket as SyncPacketV3, IsoPacket as IsoPacketV3};
use bt_hci_v3::ControllerToHostPacket as ControllerToHostPacketV3;

// Import nrf-sdc Error type
use nrf_sdc::Error as SdcError;

/// Compatibility adapter that bridges nrf-sdc (bt-hci@0.3) with trouble-host (bt-hci@0.2)
///
/// This wrapper implements `trouble_host::Controller` by delegating to the underlying
/// `SoftdeviceController` and converting types between bt-hci versions.
pub struct BleCompatController<'d> {
    inner: SoftdeviceController<'d>,
}

impl<'d> BleCompatController<'d> {
    /// Create a new compatibility adapter
    pub fn new(controller: SoftdeviceController<'d>) -> Self {
        Self { inner: controller }
    }

    /// Get a reference to the underlying controller
    pub fn inner(&self) -> &SoftdeviceController<'d> {
        &self.inner
    }

    /// Get a mutable reference to the underlying controller
    pub fn inner_mut(&mut self) -> &mut SoftdeviceController<'d> {
        &mut self.inner
    }
}

impl<'d> ErrorType for BleCompatController<'d> {
    type Error = SdcError;
}

// Helper: Convert embedded_io error to SdcError
fn convert_io_error(e: embedded_io::SliceWriteError) -> SdcError {
    match e {
        embedded_io::SliceWriteError::Full => SdcError::ENOMEM,
        _ => SdcError::EINVAL,
    }
}

// Note: We don't actually need to convert AclPacket to AclPacketV3
// since write_acl_data just sends raw bytes via hci_data_put.
// This function is kept for potential future use but is not currently called.

// Helper: Convert bt-hci@0.3 response to bt-hci@0.2 by serializing
// Works for FixedSizeValue types (which implement AsHciBytes and FromHciBytes)
fn convert_return_v3_to_v2<'de, V3, V2>(ret_v3: &'de V3) -> Result<V2, SdcError>
where
    V3: AsHciBytes,
    V2: FromHciBytes<'de>,
{
    // Get bytes from v3 response (AsHciBytes returns &[u8])
    let bytes = ret_v3.as_hci_bytes();
    // Deserialize as v2 type (FromHciBytes returns (T, &[u8]))
    let (v2, _) = V2::from_hci_bytes(bytes).map_err(|_| SdcError::EINVAL)?;
    Ok(v2)
}

// Helper: Convert bt-hci@0.2 command to bt-hci@0.3 using unsafe transmute
// Since HCI commands are binary-compatible (they follow the Bluetooth spec),
// we can safely transmute between versions if the types have the same size
fn convert_cmd_v2_to_v3<V2, V3>(cmd_v2: &V2) -> Result<V3, SdcError>
where
    V2: Sized,
    V3: Sized,
{
    // Check that both types have the same size
    let v2_size = core::mem::size_of::<V2>();
    let v3_size = core::mem::size_of::<V3>();
    if v2_size != v3_size {
        return Err(SdcError::EINVAL);
    }
    
    // Safety: HCI commands are binary-compatible between bt-hci versions
    // They follow the Bluetooth HCI specification, so the binary layout is identical
    // We've verified that both types have the same size
    Ok(unsafe { core::mem::transmute_copy(cmd_v2) })
}

// Implement bt_hci::controller::Controller trait (bt-hci@0.2)
impl<'d> BtHciController for BleCompatController<'d> {
    async fn write_acl_data(&self, packet: &AclPacket<'_>) -> Result<(), Self::Error> {
        // Serialize v2 packet to bytes and send via raw HCI interface
        // The binary format is identical between versions
        use embedded_io::Write;
        struct BufWriter {
            buf: heapless::Vec<u8, 512>,
        }
        impl embedded_io::ErrorType for BufWriter {
            type Error = embedded_io::SliceWriteError;
        }
        impl Write for BufWriter {
            fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
                for &byte in data {
                    self.buf.push(byte).map_err(|_| embedded_io::SliceWriteError::Full)?;
                }
                Ok(data.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }
        let mut writer = BufWriter { buf: heapless::Vec::new() };
        packet.write_hci(&mut writer).map_err(convert_io_error)?;
        // Convert Vec to slice for hci_data_put
        let buf_slice: &[u8] = &writer.buf;
        self.inner.hci_data_put(buf_slice)
    }

    async fn write_sync_data(&self, _packet: &SyncPacket<'_>) -> Result<(), Self::Error> {
        // Note: SoftdeviceController doesn't support sync data (returns unimplemented)
        Err(SdcError::EINVAL)
    }

    async fn write_iso_data(&self, packet: &IsoPacket<'_>) -> Result<(), Self::Error> {
        // Serialize v2 packet to bytes and send via raw HCI interface
        use embedded_io::Write;
        struct BufWriter {
            buf: heapless::Vec<u8, 512>,
        }
        impl embedded_io::ErrorType for BufWriter {
            type Error = embedded_io::SliceWriteError;
        }
        impl Write for BufWriter {
            fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
                for &byte in data {
                    self.buf.push(byte).map_err(|_| embedded_io::SliceWriteError::Full)?;
                }
                Ok(data.len())
            }
            fn flush(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
        }
        let mut writer = BufWriter { buf: heapless::Vec::new() };
        packet.write_hci(&mut writer).map_err(convert_io_error)?;
        // Convert Vec to slice for hci_iso_data_put
        let buf_slice: &[u8] = &writer.buf;
        self.inner.hci_iso_data_put(buf_slice)
    }

    async fn read<'a>(&self, buf: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        // Read from underlying controller using hci_get (returns PacketKind from bt-hci@0.3)
        // The buffer will contain the HCI packet data
        let kind_v3 = self.inner.hci_get(buf).await?;
        
        // Convert PacketKind from v3 to v2 (they're the same enum, but different types)
        use bt_hci::PacketKind as PacketKindV2;
        let kind_v2 = match kind_v3 {
            bt_hci_v3::PacketKind::Event => PacketKindV2::Event,
            bt_hci_v3::PacketKind::AclData => PacketKindV2::AclData,
            bt_hci_v3::PacketKind::SyncData => PacketKindV2::SyncData,
            bt_hci_v3::PacketKind::IsoData => PacketKindV2::IsoData,
            bt_hci_v3::PacketKind::Cmd => return Err(SdcError::EINVAL),
        };
        
        // Deserialize directly as v2 packet since the binary format is identical
        ControllerToHostPacket::from_hci_bytes_with_kind(kind_v2, buf)
            .map(|(pkt, _)| pkt)
            .map_err(|_| SdcError::EINVAL)
    }
}

// Macro to generate ControllerCmdSync implementations
// This reduces ~30 implementations to a single macro invocation per command
macro_rules! impl_cmd_sync {
    ($v2_cmd:ty, $v3_cmd:ty) => {
        impl<'d> ControllerCmdSync<$v2_cmd> for BleCompatController<'d> {
            async fn exec(
                &self,
                cmd_v2: &$v2_cmd,
            ) -> Result<<$v2_cmd as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
                // Convert v2 command to v3
                let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
                    .map_err(|e| bt_hci::cmd::Error::Io(e))?;
                
                // Execute on underlying controller using v3 trait
                // Note: ControllerCmdSyncV3::exec returns bt_hci_v3::cmd::Error<nrf_sdc::Error>
                // We need to convert it to bt_hci::cmd::Error<SdcError>
                // Use fully qualified path with explicit type annotation
                let ret_v3 = <SoftdeviceController as ControllerCmdSyncV3<$v3_cmd>>::exec(&self.inner, &cmd_v3).await
                    .map_err(|e| match e {
                        bt_hci_v3::cmd::Error::Hci(_) => {
                            bt_hci::cmd::Error::Io(SdcError::EINVAL)
                        }
                        bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
                    })?;
                
                // Convert v3 response to v2 response
                // Check if return type is () - no conversion needed
                let ret_size = core::mem::size_of::<<$v2_cmd as SyncCmd>::Return>();
                if ret_size == 0 {
                    // Return type is () - no conversion needed
                    Ok(unsafe { core::mem::zeroed() })
                } else {
                    // For FixedSizeValue types, use unsafe transmute since layout is identical
                    use bt_hci::FixedSizeValue;
                    use core::mem;
                    // Safety: HCI return types have identical binary layout between versions
                    // We need to ensure both types are the same size
                    let v3_size = core::mem::size_of_val(&ret_v3);
                    if ret_size != v3_size {
                        return Err(bt_hci::cmd::Error::Io(SdcError::EINVAL));
                    }
                    // Use unsafe transmute since types have identical layout
                    Ok(unsafe { mem::transmute_copy(&ret_v3) })
                }
            }
        }
    };
}

// Macro for commands with no parameters (like LeReadBufferSize)
macro_rules! impl_cmd_sync_no_params {
    ($v2_cmd:ty, $v3_cmd:ty) => {
        impl<'d> ControllerCmdSync<$v2_cmd> for BleCompatController<'d> {
            async fn exec(
                &self,
                _cmd: &$v2_cmd,
            ) -> Result<<$v2_cmd as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
                // Create v3 command (no parameters)
                let cmd_v3 = <$v3_cmd>::new();
                
                // Execute on underlying controller
                // Use explicit type annotation to help compiler inference
                type V3Return = <$v3_cmd as bt_hci_v3::cmd::SyncCmd>::Return;
                let ret_v3: V3Return = ControllerCmdSyncV3::exec(&self.inner, &cmd_v3).await
                    .map_err(|e| match e {
                        bt_hci_v3::cmd::Error::Hci(_) => {
                            bt_hci::cmd::Error::Io(SdcError::EINVAL)
                        }
                        bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
                    })?;
                
                // Convert v3 response to v2 response
                // Check if return type is () - no conversion needed
                let ret_size = core::mem::size_of::<<$v2_cmd as SyncCmd>::Return>();
                if ret_size == 0 {
                    // Return type is () - no conversion needed
                    Ok(unsafe { core::mem::zeroed() })
                } else {
                    // For FixedSizeValue types, use unsafe transmute since layout is identical
                    use core::mem;
                    // Safety: HCI return types have identical binary layout between versions
                    // We need to ensure both types are the same size
                    let v3_size = core::mem::size_of_val(&ret_v3);
                    if ret_size != v3_size {
                        return Err(bt_hci::cmd::Error::Io(SdcError::EINVAL));
                    }
                    // Use unsafe transmute since types have identical layout
                    Ok(unsafe { mem::transmute_copy(&ret_v3) })
                }
            }
        }
    };
}

// Macro for async commands
macro_rules! impl_cmd_async {
    ($v2_cmd:ty, $v3_cmd:ty) => {
        impl<'d> ControllerCmdAsync<$v2_cmd> for BleCompatController<'d> {
            async fn exec(
                &self,
                cmd_v2: &$v2_cmd,
            ) -> Result<(), bt_hci::cmd::Error<Self::Error>> {
                // Convert v2 command to v3
                let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
                    .map_err(|e| bt_hci::cmd::Error::Io(e))?;
                
                // Execute on underlying controller
                // Async commands return () - no need to store the result
                <SoftdeviceController as ControllerCmdAsyncV3<$v3_cmd>>::exec(&self.inner, &cmd_v3).await
                    .map_err(|e| match e {
                        bt_hci_v3::cmd::Error::Hci(_) => {
                            // Convert param error - for now, map to Io variant
                            bt_hci::cmd::Error::Io(SdcError::EINVAL)
                        }
                        bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
                    })
            }
        }
    };
}

// Implement all required command traits
// Synchronous commands
use bt_hci::cmd::le::LeReadBufferSize as LeReadBufferSizeV2;
impl_cmd_sync_no_params!(LeReadBufferSizeV2, LeReadBufferSizeV3);

// Import all other command types we need
use bt_hci::cmd::link_control::Disconnect as DisconnectV2;
use bt_hci::cmd::controller_baseband::{
    SetEventMask as SetEventMaskV2,
    SetEventMaskPage2 as SetEventMaskPage2V2,
    HostBufferSize as HostBufferSizeV2,
    SetControllerToHostFlowControl as SetControllerToHostFlowControlV2,
    Reset as ResetV2,
};
use bt_hci::cmd::status::ReadRssi as ReadRssiV2;
use bt_hci::cmd::info::ReadBdAddr as ReadBdAddrV2;
use bt_hci::cmd::le::{
    LeSetEventMask as LeSetEventMaskV2,
    LeSetRandomAddr as LeSetRandomAddrV2,
    LeReadFilterAcceptListSize as LeReadFilterAcceptListSizeV2,
    LeCreateConnCancel as LeCreateConnCancelV2,
    LeSetScanEnable as LeSetScanEnableV2,
    LeSetExtScanEnable as LeSetExtScanEnableV2,
    LeClearFilterAcceptList as LeClearFilterAcceptListV2,
    LeAddDeviceToFilterAcceptList as LeAddDeviceToFilterAcceptListV2,
    LeSetAdvEnable as LeSetAdvEnableV2,
    LeSetExtAdvEnable as LeSetExtAdvEnableV2,
    LeSetAdvData as LeSetAdvDataV2,
    LeSetAdvParams as LeSetAdvParamsV2,
    LeSetScanResponseData as LeSetScanResponseDataV2,
    LeLongTermKeyRequestReply as LeLongTermKeyRequestReplyV2,
    LeConnUpdate as LeConnUpdateV2,
    LeCreateConn as LeCreateConnV2,
    LeEnableEncryption as LeEnableEncryptionV2,
};
use bt_hci::cmd::controller_baseband::HostNumberOfCompletedPackets as HostNumberOfCompletedPacketsV2;

// Import v3 equivalents
use bt_hci_v3::cmd::link_control::Disconnect as DisconnectV3;
use bt_hci_v3::cmd::controller_baseband::{
    SetEventMask as SetEventMaskV3,
    SetEventMaskPage2 as SetEventMaskPage2V3,
    HostBufferSize as HostBufferSizeV3,
    SetControllerToHostFlowControl as SetControllerToHostFlowControlV3,
    Reset as ResetV3,
    HostNumberOfCompletedPackets as HostNumberOfCompletedPacketsV3,
};
use bt_hci_v3::cmd::status::ReadRssi as ReadRssiV3;
use bt_hci_v3::cmd::info::ReadBdAddr as ReadBdAddrV3;
use bt_hci_v3::cmd::le::{
    LeSetEventMask as LeSetEventMaskV3,
    LeSetRandomAddr as LeSetRandomAddrV3,
    LeReadFilterAcceptListSize as LeReadFilterAcceptListSizeV3,
    LeCreateConnCancel as LeCreateConnCancelV3,
    LeSetScanEnable as LeSetScanEnableV3,
    LeSetExtScanEnable as LeSetExtScanEnableV3,
    LeClearFilterAcceptList as LeClearFilterAcceptListV3,
    LeAddDeviceToFilterAcceptList as LeAddDeviceToFilterAcceptListV3,
    LeSetAdvEnable as LeSetAdvEnableV3,
    LeSetExtAdvEnable as LeSetExtAdvEnableV3,
    LeSetAdvData as LeSetAdvDataV3,
    LeSetAdvParams as LeSetAdvParamsV3,
    LeSetScanResponseData as LeSetScanResponseDataV3,
    LeLongTermKeyRequestReply as LeLongTermKeyRequestReplyV3,
    LeConnUpdate as LeConnUpdateV3,
    LeCreateConn as LeCreateConnV3,
    LeEnableEncryption as LeEnableEncryptionV3,
};

// Implement all synchronous commands
impl_cmd_sync!(DisconnectV2, DisconnectV3);
impl_cmd_sync!(SetEventMaskV2, SetEventMaskV3);
impl_cmd_sync!(SetEventMaskPage2V2, SetEventMaskPage2V3);
impl_cmd_sync!(LeSetEventMaskV2, LeSetEventMaskV3);
impl_cmd_sync!(LeSetRandomAddrV2, LeSetRandomAddrV3);
impl_cmd_sync!(HostBufferSizeV2, HostBufferSizeV3);
impl_cmd_sync!(LeReadFilterAcceptListSizeV2, LeReadFilterAcceptListSizeV3);
impl_cmd_sync!(SetControllerToHostFlowControlV2, SetControllerToHostFlowControlV3);
impl_cmd_sync!(ResetV2, ResetV3);
impl_cmd_sync!(ReadRssiV2, ReadRssiV3);
impl_cmd_sync!(LeCreateConnCancelV2, LeCreateConnCancelV3);
impl_cmd_sync!(LeSetScanEnableV2, LeSetScanEnableV3);
impl_cmd_sync!(LeSetExtScanEnableV2, LeSetExtScanEnableV3);
impl_cmd_sync!(LeClearFilterAcceptListV2, LeClearFilterAcceptListV3);
impl_cmd_sync!(LeAddDeviceToFilterAcceptListV2, LeAddDeviceToFilterAcceptListV3);
impl_cmd_sync!(LeSetAdvParamsV2, LeSetAdvParamsV3);
impl_cmd_sync!(LeLongTermKeyRequestReplyV2, LeLongTermKeyRequestReplyV3);
impl_cmd_sync!(ReadBdAddrV2, ReadBdAddrV3);

// Commands with lifetime parameters need special handling
// For now, we'll implement them manually since the macro doesn't handle lifetimes well
impl<'d> ControllerCmdSync<LeSetAdvEnableV2> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &LeSetAdvEnableV2,
    ) -> Result<<LeSetAdvEnableV2 as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
            .map_err(|e| bt_hci::cmd::Error::Io(e))?;
        let _ret_v3: <LeSetAdvEnableV3 as bt_hci_v3::cmd::SyncCmd>::Return = <SoftdeviceController as ControllerCmdSyncV3<LeSetAdvEnableV3>>::exec(&self.inner, &cmd_v3).await
            .map_err(|e| match e {
                bt_hci_v3::cmd::Error::Hci(_) => {
                    // Convert param error - for now, map to Io variant
                    bt_hci::cmd::Error::Io(SdcError::EINVAL)
                }
                bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
            })?;
        // LeSetAdvEnable returns () - no conversion needed
        Ok(())
    }
}

impl<'d, 't> ControllerCmdSync<LeSetExtAdvEnableV2<'t>> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &LeSetExtAdvEnableV2<'t>,
    ) -> Result<<LeSetExtAdvEnableV2<'t> as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
            .map_err(|e| bt_hci::cmd::Error::Io(e))?;
        let _ret_v3: <LeSetExtAdvEnableV3 as bt_hci_v3::cmd::SyncCmd>::Return = <SoftdeviceController as ControllerCmdSyncV3<LeSetExtAdvEnableV3>>::exec(&self.inner, &cmd_v3).await
            .map_err(|e| match e {
                bt_hci_v3::cmd::Error::Hci(_) => {
                    // Convert param error - for now, map to Io variant
                    bt_hci::cmd::Error::Io(SdcError::EINVAL)
                }
                bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
            })?;
        // LeSetExtAdvEnable returns () - no conversion needed
        Ok(())
    }
}

impl<'d, 't> ControllerCmdSync<HostNumberOfCompletedPacketsV2<'t>> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &HostNumberOfCompletedPacketsV2<'t>,
    ) -> Result<<HostNumberOfCompletedPacketsV2<'t> as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
            .map_err(|e| bt_hci::cmd::Error::Io(e))?;
        let _ret_v3: <HostNumberOfCompletedPacketsV3 as bt_hci_v3::cmd::SyncCmd>::Return = <SoftdeviceController as ControllerCmdSyncV3<HostNumberOfCompletedPacketsV3>>::exec(&self.inner, &cmd_v3).await
            .map_err(|e| match e {
                bt_hci_v3::cmd::Error::Hci(_) => {
                    // Convert param error - for now, map to Io variant
                    bt_hci::cmd::Error::Io(SdcError::EINVAL)
                }
                bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
            })?;
        // HostNumberOfCompletedPackets returns () - no conversion needed
        Ok(())
    }
}

// LeSetAdvData and LeSetScanResponseData don't have lifetime parameters in v2
impl<'d> ControllerCmdSync<LeSetAdvDataV2> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &LeSetAdvDataV2,
    ) -> Result<<LeSetAdvDataV2 as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
            .map_err(|e| bt_hci::cmd::Error::Io(e))?;
        let _ret_v3: <LeSetAdvEnableV3 as bt_hci_v3::cmd::SyncCmd>::Return = <SoftdeviceController as ControllerCmdSyncV3<LeSetAdvEnableV3>>::exec(&self.inner, &cmd_v3).await
            .map_err(|e| match e {
                bt_hci_v3::cmd::Error::Hci(_) => {
                    // Convert param error - for now, map to Io variant
                    bt_hci::cmd::Error::Io(SdcError::EINVAL)
                }
                bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
            })?;
        // LeSetAdvData returns () - no conversion needed
        Ok(())
    }
}

impl<'d> ControllerCmdSync<LeSetScanResponseDataV2> for BleCompatController<'d> {
    async fn exec(
        &self,
        cmd_v2: &LeSetScanResponseDataV2,
    ) -> Result<<LeSetScanResponseDataV2 as SyncCmd>::Return, bt_hci::cmd::Error<Self::Error>> {
        let cmd_v3 = convert_cmd_v2_to_v3(cmd_v2)
            .map_err(|e| bt_hci::cmd::Error::Io(e))?;
        let _ret_v3: <LeSetAdvEnableV3 as bt_hci_v3::cmd::SyncCmd>::Return = <SoftdeviceController as ControllerCmdSyncV3<LeSetAdvEnableV3>>::exec(&self.inner, &cmd_v3).await
            .map_err(|e| match e {
                bt_hci_v3::cmd::Error::Hci(_) => {
                    // Convert param error - for now, map to Io variant
                    bt_hci::cmd::Error::Io(SdcError::EINVAL)
                }
                bt_hci_v3::cmd::Error::Io(e) => bt_hci::cmd::Error::Io(e),
            })?;
        // LeSetScanResponseData returns () - no conversion needed
        Ok(())
    }
}

// Implement async commands
impl_cmd_async!(LeConnUpdateV2, LeConnUpdateV3);
impl_cmd_async!(LeCreateConnV2, LeCreateConnV3);
impl_cmd_async!(LeEnableEncryptionV2, LeEnableEncryptionV3);

// Implement trouble_host::Controller
// This trait is automatically implemented via trait bounds if we implement all required command traits
// No explicit impl needed - it's a marker trait
