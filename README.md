# JTKJ24_morsetin_ryhmaprojekti
Mikrokontrolleria liikuttamalla lähetetään morsekoodia UART:n välityksellä käyttäen gyroskooppianturia ja RTOS-käyttöjärjestelmää.
---
# Toiminta
1. Luetaan liike ja kiihtyvyysanturilta dataa ja muutetaan se morsekoodiksi. Tulostetaan käyttäjälle tekstinä
2. Käytetään MPU 9250 liikeanturia. Alustetaan sensori. Kerätään data sensortaskilla ja muutetaan se morsekoodiksi. Uart-taskissa lähetetään data terminaaliin morsekoodina.  
3. UART-taskilla siirretään dataa laitteelta näytölle. SensorTask kerää dataa vain WAITING tilassa.  
4. Liike ylöspäin ja alaspäin on piste ja liike vaakasuunnassa pöydällä on viiva. 
5. Datan keräys 10hz taajuudella (100ms viive) 
6. Ledin palaessa seuraavan liikkeen voi suorittaa. Napin painalluksella datan keräys käynnistyy
---
# Tilakone
<img width="387" height="274" alt="Näyttökuva 2025-11-02 162903" src="https://github.com/user-attachments/assets/00efa44f-f4b1-4a0a-a29c-fb5ea080b0c0" />
---
### Huom.
Sisältää vain päätiedoston main.c
