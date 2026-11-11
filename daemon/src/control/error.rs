//! Request failures and the gRPC status each maps to.

use tonic::{Code, Status};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    /// A parameter is missing, out of its type's range, or names no value.
    BadRequest,
    /// No such stream, antenna or channel.
    NotFound,
    /// Another session holds control, or the request conflicts with a running
    /// stream.
    Busy,
    /// A value outside what the radio accepts, such as a sample rate.
    OutOfRange,
    /// The transceiver refused the request.
    Radio,
    /// A stream could not be opened, started or stopped.
    Stream,
    Internal,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ApiError {
    pub code: ErrorCode,
    pub message: String,
}

impl ApiError {
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }

    pub fn bad_request(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::BadRequest, message)
    }
}

impl std::fmt::Display for ApiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ApiError {}

impl From<ApiError> for Status {
    fn from(error: ApiError) -> Self {
        let code = match error.code {
            ErrorCode::BadRequest => Code::InvalidArgument,
            ErrorCode::NotFound => Code::NotFound,
            ErrorCode::Busy => Code::FailedPrecondition,
            ErrorCode::OutOfRange => Code::OutOfRange,
            ErrorCode::Radio => Code::Unavailable,
            ErrorCode::Stream => Code::Aborted,
            ErrorCode::Internal => Code::Internal,
        };
        Status::new(code, error.message)
    }
}
