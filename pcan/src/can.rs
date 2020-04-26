//! Controller Area Network

pub trait Id {
    /// Creates a new standard (11bit) identifier
    ///
    /// Implementors are recommended to panic if the `id` parameter is not in
    /// the range of 0..0x7FF.
    fn new_standard(id: u32) -> Self;

    /// Creates a new standard (29bit) identifier
    ///
    /// Implementors are recommended to panic if the `id` parameter is not in
    /// the range of 0..0x1FFFFFFF.
    fn new_extended(id: u32) -> Self;

    fn id(&self) -> u32;

    /// Returns true if this is an extended identifier
    fn is_extended(&self) -> bool;

    /// Returns true if this is a standard identifier
    fn is_standard(&self) -> bool {
        !self.is_extended()
    }
}

/// A CAN2.0 Frame
pub trait Frame: Sized {
    type Id: Id;

    /// Creates a new data frame.
    ///
    /// Implementors are recommended to panic if the `data` slice contains more
    /// than 8 bytes.
    fn new(id: Self::Id, data: &[u8]) -> Self;

    /// TODO
    fn new_standard(id: u32, data: &[u8]) -> Self {
        Self::new(Self::Id::new_standard(id), data)
    }

    /// TODO
    fn new_extended(id: u32, data: &[u8]) -> Self {
        Self::new(Self::Id::new_extended(id), data)
    }

    /// Creates a new remote frame.
    fn new_remote(id: Self::Id) -> Self;

    /// Returns true if this `Frame` is a extended frame
    fn is_extended(&self) -> bool;

    /// Returns true if this `Frame` is a standard frame
    fn is_standard(&self) -> bool {
        !self.is_extended()
    }

    /// Returns true if this `Frame` is a remote frame
    fn is_remote_frame(&self) -> bool;

    /// Returns true if this `Frame` is a data frame
    fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }

    /// Returns the Can-ID
    fn id(&self) -> u32;

    /// Returns frame data
    fn data(&self) -> &[u8];
}

/// A CAN interface that is able to transmit frames.
pub trait Transmitter {
    type Frame: Frame;
    type Error;

    /// Put a `Frame` in the transmit buffer (or a free mailbox).
    ///
    /// If the buffer is full, this function will try to replace a lower priority `Frame`
    /// and return it. This is to avoid the priority inversion problem.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error>;
}

/// A CAN interface that is able to receive frames.
pub trait Receiver {
    type Frame: Frame;
    type Error;

    /// Return the available `Frame` with the highest priority (lowest ID).
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error>;
}

pub trait MessageFilter: Receiver {
    type FilterId: Id;

    /// Creates a new filter
    fn add_filter(&mut self, id: Self::FilterId) -> Result<(), Self::Error>;

    /// Creates a new filter with mask
    fn add_filter_with_mask(
        &mut self,
        id: Self::FilterId,
        mask: Self::FilterId,
    ) -> Result<(), Self::Error>;

    fn clear_filters(&mut self);
}
