/**
 * i18n.js - UI language switching (English / Chinese)
 *
 * The UI is authored in English. Switching language swaps the text of every
 * DOM text node and title/placeholder attribute whose (whitespace-collapsed)
 * English content has an entry in the language dictionary, and a
 * MutationObserver keeps doing that for text the controllers write later
 * (button states, status lines, dynamically built panels). Strings without
 * an entry — telemetry values, parameter names, flight-mode names, units —
 * pass through untouched, so nothing has to be annotated in the markup.
 *
 * Each translated node remembers the English it replaced, so switching back
 * restores the exact original rather than a reverse-dictionary guess.
 *
 * Canvas HUD text and documentation stay English by design.
 */

import { ZH } from './lang-zh.js';

const LANG_KEY = 'gcs-lang';
const DICTS = { en: null, zh: ZH };
const ATTRS = ['title', 'placeholder', 'aria-label'];
const SKIP_TAGS = new Set(['SCRIPT', 'STYLE', 'TEXTAREA', 'CODE', 'PRE']);

let lang = 'en';
let observer = null;

export function getLanguage() {
    return lang;
}

/** Translate a single string for the active language (dictionary miss → input). */
export function t(text) {
    const dict = DICTS[lang];
    return (dict && dict[text]) || text;
}

const collapse = s => s.replace(/\s+/g, ' ').trim();

function translateTextNode(node) {
    const raw = node.nodeValue;
    if (!raw || !/\S/.test(raw)) return;
    const parent = node.parentNode;
    if (!parent || SKIP_TAGS.has(parent.nodeName)) return;

    const dict = DICTS[lang];
    if (!dict) {
        // Back to English: undo only what we wrote and nothing has touched since
        if (node.__i18nOut !== undefined && node.__i18nOut === raw) node.nodeValue = node.__i18nSrc;
        return;
    }
    if (node.__i18nOut === raw) return;              // already ours
    const tr = dict[collapse(raw)];
    if (!tr) return;
    const lead = raw.match(/^\s*/)[0];
    const trail = raw.match(/\s*$/)[0];
    const out = lead + tr + trail;
    node.__i18nSrc = raw;
    node.__i18nOut = out;
    node.nodeValue = out;
}

function translateAttr(el, name) {
    const raw = el.getAttribute(name);
    if (!raw) return;
    const store = el.__i18nAttr || (el.__i18nAttr = {});
    const dict = DICTS[lang];
    if (!dict) {
        if (store[name] && store[name].out === raw) el.setAttribute(name, store[name].src);
        return;
    }
    if (store[name] && store[name].out === raw) return;
    const tr = dict[collapse(raw)];
    if (!tr) return;
    store[name] = { src: raw, out: tr };
    el.setAttribute(name, tr);
}

function walk(root) {
    if (root.nodeType === Node.TEXT_NODE) { translateTextNode(root); return; }
    if (root.nodeType !== Node.ELEMENT_NODE && root.nodeType !== Node.DOCUMENT_FRAGMENT_NODE) return;
    if (root.nodeType === Node.ELEMENT_NODE) {
        if (SKIP_TAGS.has(root.nodeName)) return;
        for (const a of ATTRS) if (root.hasAttribute(a)) translateAttr(root, a);
    }
    const walker = document.createTreeWalker(root, NodeFilter.SHOW_ELEMENT | NodeFilter.SHOW_TEXT);
    let n;
    while ((n = walker.nextNode())) {
        if (n.nodeType === Node.TEXT_NODE) translateTextNode(n);
        else if (!SKIP_TAGS.has(n.nodeName)) {
            for (const a of ATTRS) if (n.hasAttribute(a)) translateAttr(n, a);
        }
    }
}

function onMutations(records) {
    for (const r of records) {
        if (r.type === 'characterData') translateTextNode(r.target);
        else if (r.type === 'attributes') translateAttr(r.target, r.attributeName);
        else for (const n of r.addedNodes) walk(n);
    }
}

function startObserver() {
    if (observer) return;
    observer = new MutationObserver(onMutations);
    observer.observe(document.body, {
        childList: true, subtree: true, characterData: true,
        attributes: true, attributeFilter: ATTRS,
    });
}

function stopObserver() {
    if (!observer) return;
    observer.disconnect();
    observer = null;
}

/**
 * Switch the UI language ('en' | 'zh'), persist it, and re-render every
 * translatable node in place.
 */
export function setLanguage(next) {
    if (!(next in DICTS)) next = 'en';
    lang = next;
    try { localStorage.setItem(LANG_KEY, lang); } catch (e) { /* ignore */ }
    document.documentElement.lang = lang === 'zh' ? 'zh-CN' : 'en';
    // Restore pass runs with the observer off so it doesn't chase its own writes
    stopObserver();
    walk(document.body);
    if (DICTS[lang]) startObserver();
    const sel = document.getElementById('lang-select');
    if (sel && sel.value !== lang) sel.value = lang;
    window.dispatchEvent(new CustomEvent('languageChanged', { detail: { lang } }));
}

/** Apply the saved language. Call before the controllers build their DOM. */
export function initI18n() {
    let saved = 'en';
    try { saved = localStorage.getItem(LANG_KEY) || 'en'; } catch (e) { /* ignore */ }
    setLanguage(saved);
}
