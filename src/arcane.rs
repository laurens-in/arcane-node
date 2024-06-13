use embedded_hal::can::{Id, StandardId};

/// The ARCANE id
pub struct ArcaneId(u16);

impl ArcaneId {
    /// Creates a new `ArcaneId` from a given function code and node ID.
    ///
    /// # Parameters
    ///
    /// - `function_code`: The function code of type `ArcaneCode`.
    /// - `node_id`: The node ID, which must be a 7-bit value (0 to 0x7F).
    ///
    /// # Returns
    ///
    /// - `Some(ArcaneId)` if the node ID is valid.
    /// - `None` if the node ID is out of range.
    ///
    /// # Example
    ///
    /// ```
    /// let arcane_id = ArcaneId::new(ArcaneCode::NMT, 0x01);
    /// assert!(arcane_id.is_some());
    /// ```
    pub fn new(function_code: ArcaneCode, node_id: u8) -> Option<Self> {
        if node_id <= 0x7F {
            Some(ArcaneId(((function_code as u16) << 7) | (node_id as u16)))
        } else {
            None
        }
    }

    /// Returns the raw 16-bit value of the `ArcaneId`.
    ///
    /// # Returns
    ///
    /// - The raw 16-bit value.
    ///
    /// # Example
    ///
    /// ```
    /// let arcane_id = ArcaneId::new(ArcaneCode::NMT, 0x01).unwrap();
    /// assert_eq!(arcane_id.as_raw(), 0x0001);
    /// ```
    pub fn as_raw(&self) -> u16 {
        self.0
    }

    /// Converts the `ArcaneId` to a CAN `Id`.
    ///
    /// # Returns
    ///
    /// - The CAN `Id` as an `Id::Standard`.
    ///
    /// # Example
    ///
    /// ```
    /// let arcane_id = ArcaneId::new(ArcaneCode::NMT, 0x01).unwrap();
    /// let can_id = arcane_id.as_can_id();
    /// ```
    pub fn as_can_id(&self) -> Id {
        Id::Standard(self.as_standard_id())
    }

    /// Converts the `ArcaneId` to a CAN `StandardId`.
    ///
    /// # Returns
    ///
    /// - The CAN `StandardId`.
    ///
    ///
    /// # Example
    ///
    /// ```
    /// let arcane_id = ArcaneId::new(ArcaneCode::NMT, 0x01).unwrap();
    /// let standard_id = arcane_id.as_standard_id();
    /// ```
    pub fn as_standard_id(&self) -> StandardId {
        StandardId::new(self.0).unwrap()
    }
}

/// Enum representing different function codes in the ARCANE system.
pub enum ArcaneCode {
    NMT = 0x0,
    MIDI0 = 0x1,
    RAW0 = 0x2,
    MIDI1 = 0x3,
    RAW1 = 0x4,
    MIDI2 = 0x5,
    RAW2 = 0x6,
    CFGW = 0x7,
    CFGR = 0x8,
}
