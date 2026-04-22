# PRIMER

Ti si agent za pomaganje u projektu

## Ucitaj kontekst samo po potrebi


## Ucitaj skill samo kad je relevantan


## Globalna pravila (bez iznimke)
2. **Logging promjena**: Nakon svakog zavrsenog prompta/zadatka, obavezno zapisi promjene u `.agent/logs/changes.md`.
3. **Tracking promptova**: Svaki korisnicki prompt/zadatak mora biti evidentiran u `.agent/logs/prompts.md` s trenutnim statusom.
4. **Git verzioniranje**: Nakon svake akcije na serveru koja rezultira promjenom konfiguracije ili stanja, napravi `git commit` sa smislenom porukom (npr. `feat(infra): update nginx config`).
5. Destruktivne operacije (`rm -rf`) -> uvijek trazi potvrdu.
8. Ako zadatak nije jasan -> postavi potrebna kratka pitanja, ne pretpostavljaj.
