import base64
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives.hashes import SHA256
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC


class AESCipher:
    """
    A wrapper around the true python AESCipher.

    This wrapper takes care of properly padding messages, selection an encryption mode and handling the encryption key.
    It is safe to keep one object because the internal python cipher is not reused.
    """

    def __init__(self, key: str):
        """
        :param key: The passphrase used to encrypt and decrypt messages.
            If it is None, no encryption/decryption takes place
        """
        if key is not None and len(key) == 0:
            key = None

        if key is None:
            self.key = None
        else:
            # hash the key so that it has a fixed length and is urlsafe as required by the Fernet cipher
            key = PBKDF2HMAC(
                algorithm=SHA256(),
                length=32,
                salt=b"bit-bots",
                iterations=10000
            ).derive(bytes(key, encoding="UTF-8"))
            self.key = base64.urlsafe_b64encode(key)

    def encrypt(self, message: str) -> bytes:
        if message == "":
            raise ValueError("Cannot encrypt empty message")
        if self.key is None:
            return bytes(message, encoding="UTF-8")

        return Fernet(key=self.key).encrypt(bytes(message, encoding="UTF-8"))

    def decrypt(self, enc: bytes) -> str:
        if len(enc) == 0:
            raise ValueError("Cannot decrypt empty data")
        if self.key is None:
            return str(enc, encoding="UTF-8")

        return str(Fernet(key=self.key).decrypt(enc), encoding="UTF-8")
