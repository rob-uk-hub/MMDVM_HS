class CHamming {
    public:
        CHamming();
        bool Hamming_7_4_decode(unsigned char *rxBits);
        bool QR_16_7_6_decode(unsigned char *rxBits);


    private:
        void Hamming_7_4_init();
        void QR_16_7_6_init();

        //!< Parity check matrix of bits
        const unsigned char Hamming_7_4_m_H[7*3] = {
            1, 1, 1, 0,   1, 0, 0,
            0, 1, 1, 1,   0, 1, 0,
            1, 1, 0, 1,   0, 0, 1
        //  0  1  2  3 <- correctable bit positions
        };

        const unsigned char QR_16_7_6_m_H[16*9] = {
            0, 1, 1,  1, 1, 0, 0,   1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1,  1, 1, 1, 0,   0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 0,  1, 1, 1, 1,   0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1,  1, 0, 1, 1,   0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 1, 1,  0, 0, 0, 1,   0, 0, 0, 0, 1, 0, 0, 0, 0,
            1, 1, 0,  0, 1, 0, 0,   0, 0, 0, 0, 0, 1, 0, 0, 0,
            1, 1, 1,  0, 0, 1, 0,   0, 0, 0, 0, 0, 0, 1, 0, 0,
            1, 1, 1,  1, 0, 0, 1,   0, 0, 0, 0, 0, 0, 0, 1, 0,
            1, 0, 1,  0, 1, 1, 1,   0, 0, 0, 0, 0, 0, 0, 0, 1,
        };

        unsigned char Hamming_7_4_m_corr[8]; //!< single bit error correction by syndrome index
        unsigned char QR_16_7_6_m_corr[512][2]; //!< up to 2 bit error correction by syndrome index
};
